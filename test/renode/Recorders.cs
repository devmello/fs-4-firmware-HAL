//
// Monitor commands for the VCU tests. Everything is stamped with virtual time
// in microseconds (Renode's own log only has host time).
//
// Recording
//   can1 RecordFrames "P" @can.txt            <us> P <id hex> <s|x><d|r> <data hex, or ->
//   uart4 RecordLines @uart.txt               <us> <line>
//   uart4 FlushLine                           writes a line still missing its "\n"
//   gpioPortC RecordPin 0 "PC0" @pins.txt     <us> PC0 <0|1>, on every change
//   machine SampleBytes "a=0x2000.. b=.." 100 @ram.txt   <us> a <value>, polled every 100 us
//   machine EmulateResetFlags @resets.txt     RCC_CSR reset flags, see below ("" for no log)
//   cpu RecordWordAt <pc> <address> "x" @f.txt     <us> x <32-bit value>, each time pc runs
//   cpu RecordRegisterAt <pc> <n> @f.txt      <us> <rn>, each time pc runs
//
// Input
//   can1 InjectFrame 0x391 "44"               receives a frame now (data in hex, "" for none)
//   machine LoadTimeline @in.txt @events.txt  inputs at given virtual times, see Timeline
//
// Faults
//   cpu ReturnBetween <addr> <r0> <from us> <to us>
//       the function at <addr> returns <r0> without running, inside the window
//   cpu WriteWordAt <pc> <address> <value>    sysbus write each time pc runs
//   machine BlockWatchdogRefresh <from us> <to us>   IWDG reload keys are dropped
//   can1 HoldMailboxes <from us> <to us>      the TX mailboxes look busy
// CPU hooks are only for code that runs rarely (init, error paths). One that
// fires every loop pass makes Renode about 20 times slower.
//

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CAN;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Testing
{
    public static class VirtualTimeRecorders
    {
        // Both buses can share a file, so the order of frames across them is kept
        public static void RecordFrames(this ICAN can, string bus, string path)
        {
            var machine = MachineOf(can);
            var writer = Writer(path);
            can.FrameSent += frame => writer.WriteLine($"{Now(machine)} {bus} {FrameText(frame)}");
        }

        public static void InjectFrame(this ICAN can, uint id, string dataHex, bool extended = false, bool remote = false)
        {
            can.OnFrameReceived(new CANMessageFrame(id, Hex(dataHex), extended, remote));
        }

        public static void RecordLines(this IUART uart, string path)
        {
            var machine = MachineOf(uart);
            var line = new UartLine { Writer = Writer(path) };
            uartLines[uart] = line;
            uart.CharReceived += c =>
            {
                if(c == '\n')
                {
                    line.Writer.WriteLine($"{Now(machine)} {line.Text.ToString().TrimEnd('\r')}");
                    line.Text.Clear();
                }
                else
                {
                    line.Text.Append((char)c);
                }
            };
        }

        public static void FlushLine(this IUART uart)
        {
            if(uartLines.TryGetValue(uart, out var line) && line.Text.Length > 0)
            {
                line.Writer.WriteLine($"{Now(MachineOf(uart))} {line.Text}");
                line.Text.Clear();
            }
        }

        public static void RecordPin(this INumberedGPIOOutput port, int pin, string name, string path)
        {
            var machine = MachineOf(port);
            var writer = Writer(path);
            if(!port.Connections.TryGetValue(pin, out var gpio) || !(gpio is IGPIOWithHooks hooks))
            {
                throw new RecoverableException($"No pin {pin} on this port");
            }
            hooks.AddStateChangedHook(value => writer.WriteLine($"{Now(machine)} {name} {(value ? 1 : 0)}"));
        }

        // Reads bytes every <periodUs> and logs each change: "<us> <label> <value>".
        // spec is "label=0xADDRESS label=0xADDRESS ...". Much cheaper than a
        // watchpoint, which makes the CPU trap every access to that page.
        public static void SampleBytes(this IMachine machine, string spec, ulong periodUs, string path)
        {
            var writer = Writer(path);
            var bytes = spec.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries)
                .Select(s => s.Split('='))
                .Select(p => new { Label = p[0], Address = Convert.ToUInt64(p[1], 16) })
                .ToArray();
            var last = bytes.Select(_ => -1).ToArray();
            Action<TimeInterval> sample = null;
            sample = _ =>
            {
                var now = Now(machine);
                for(var i = 0; i < bytes.Length; i++)
                {
                    var value = machine.SystemBus.ReadByte(bytes[i].Address);
                    if(value != last[i])
                    {
                        last[i] = value;
                        writer.WriteLine($"{now} {bytes[i].Label} {value}");
                    }
                }
                machine.ScheduleAction(TimeInterval.FromMicroseconds(periodUs), sample, "sampler");
            };
            machine.ScheduleAction(TimeInterval.FromMicroseconds(periodUs), sample, "sampler");
        }

        // Some registers only read back in certain modes (CAN BTR only in init
        // mode), so read them when the firmware reaches a given function
        public static void RecordWordAt(this ICpuSupportingGdb cpu, ulong pc, ulong address, string label, string path)
        {
            var machine = MachineOf(cpu);
            var writer = Writer(path);
            cpu.AddHook(pc & ~1UL, (c, _) => writer.WriteLine($"{Now(machine)} {label} {machine.SystemBus.ReadDoubleWord(address):X8}"));
        }

        public static void RecordRegisterAt(this ICpuSupportingGdb cpu, ulong pc, int register, string path)
        {
            var machine = MachineOf(cpu);
            var writer = Writer(path);
            cpu.AddHook(pc & ~1UL, (c, _) => writer.WriteLine($"{Now(machine)} {c.GetRegister(register).RawValue}"));
        }

        public static void ReturnBetween(this ICpuSupportingGdb cpu, ulong address, ulong value, ulong fromUs, ulong toUs)
        {
            var machine = MachineOf(cpu);
            cpu.AddHook(address & ~1UL, (c, pc) =>
            {
                var now = Now(machine);
                if(now >= fromUs && now < toUs)
                {
                    var lr = c.GetRegister(14).RawValue;
                    c.SetRegister(0, RegisterValue.Create(value, 32));
                    c.PC = RegisterValue.Create(lr, 32);
                }
            });
        }

        // Reload keys written to the IWDG inside the window are dropped, as if
        // watchdog_refresh() returned early
        public static void BlockWatchdogRefresh(this IMachine machine, ulong fromUs, ulong toUs)
        {
            if(!machine.TryGetByName<IBusPeripheral>("sysbus.iwdg", out var iwdg))
            {
                throw new RecoverableException("sysbus.iwdg not found");
            }
            machine.SystemBus.SetHookBeforePeripheralWrite<uint>(iwdg, (value, offset) =>
            {
                var now = Now(machine);
                return (offset & 0xFF) == 0 && value == 0xAAAA && now >= fromUs && now < toUs ? 0u : value;
            });
        }

        // Inside the window all three TX mailboxes read as busy (TSR.TME0-2
        // clear), like a bus that never lets the frames out. Can be called
        // several times for one controller.
        public static void HoldMailboxes(this ICAN can, ulong fromUs, ulong toUs)
        {
            var machine = MachineOf(can);
            if(!holds.TryGetValue(can, out var windows))
            {
                windows = new List<Tuple<ulong, ulong>>();
                holds[can] = windows;
                machine.SystemBus.SetHookAfterPeripheralRead<uint>((IBusPeripheral)can, (value, offset) =>
                {
                    if((offset & 0x3FF) != 0x08)
                    {
                        return value;
                    }
                    var now = Now(machine);
                    return windows.Any(w => now >= w.Item1 && now < w.Item2) ? value & ~(7u << 26) : value;
                });
            }
            windows.Add(Tuple.Create(fromUs, toUs));
        }

        public static void WriteWordAt(this ICpuSupportingGdb cpu, ulong pc, ulong address, uint value)
        {
            var machine = MachineOf(cpu);
            cpu.AddHook(pc & ~1UL, (c, _) => machine.SystemBus.WriteDoubleWord(address, value));
        }

        // Renode's RCC keeps CSR at its power-on value (PORRSTF, PINRSTF,
        // BORRSTF) and ignores RMVF, and its IWDG resets the machine without
        // telling the RCC. This keeps the reset flags like the chip: RMVF
        // clears them, every reset sets PINRSTF, an IWDG reset sets IWDGRSTF.
        // Each reset is logged as "<us> reset <flags hex>" if path isn't "".
        public static void EmulateResetFlags(this IMachine machine, string path)
        {
            var writer = path == "" ? null : Writer(path);
            if(!machine.TryGetByName<IPeripheral>("sysbus.rcc", out var rccPeripheral)
               || !(rccPeripheral is IProvidesRegisterCollection<DoubleWordRegisterCollection> rcc))
            {
                throw new RecoverableException("sysbus.rcc not found");
            }
            if(!machine.TryGetByName<STM32_IndependentWatchdog>("sysbus.iwdg", out var iwdg))
            {
                throw new RecoverableException("sysbus.iwdg not found");
            }
            var timer = (LimitTimer)typeof(STM32_IndependentWatchdog)
                .GetField("watchdogTimer", BindingFlags.NonPublic | BindingFlags.Instance).GetValue(iwdg);

            var flags = PowerOnFlags;
            var watchdogFired = false;
            timer.LimitReached += () => watchdogFired = true;
            rcc.RegistersCollection.AddAfterReadHook(CsrOffset, (offset, value) => (uint?)((value & 0x00FFFFFFu) | flags));
            rcc.RegistersCollection.AddBeforeWriteHook(CsrOffset, (offset, value) =>
            {
                if((value & Rmvf) != 0)
                {
                    flags = 0;
                }
                // The model's flag bits are tags; leave them as they are
                return (value & 0x00FFFFFFu) | PowerOnFlags;
            });
            machine.MachineReset += m =>
            {
                flags |= Pinrstf | (watchdogFired ? Iwdgrstf : 0);
                watchdogFired = false;
                writer?.WriteLine($"{Now(machine)} reset {flags:X8}");
            };
        }

        public static void LoadTimeline(this IMachine machine, string path, string eventsPath)
        {
            new Timeline(machine, path, Writer(eventsPath)).Start();
        }

        public static ulong Now(IMachine machine)
        {
            // An access from the CPU is part way through its time slice
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                cpu.SyncTime();
            }
            return (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
        }

        public static IMachine MachineOf(IPeripheral peripheral)
        {
            if(!EmulationManager.Instance.CurrentEmulation.TryGetMachineForPeripheral(peripheral, out var machine))
            {
                throw new RecoverableException("Peripheral is not attached to a machine");
            }
            return machine;
        }

        public static byte[] Hex(string hex)
        {
            if(hex == null || hex == "-")
            {
                return new byte[0];
            }
            return Enumerable.Range(0, hex.Length / 2).Select(i => Convert.ToByte(hex.Substring(2 * i, 2), 16)).ToArray();
        }

        private static string FrameText(CANMessageFrame frame)
        {
            // Renode's bxCAN leaves Data null for a frame with DLC 0
            var data = frame.Data == null || frame.Data.Length == 0 ? "-" : string.Concat(frame.Data.Select(b => b.ToString("X2")));
            return $"{frame.Id:X3} {(frame.ExtendedFormat ? 'x' : 's')}{(frame.RemoteFrame ? 'r' : 'd')} {data}";
        }

        // Several recorders can share a file
        private static StreamWriter Writer(string path)
        {
            path = Path.GetFullPath(path);
            if(!writers.TryGetValue(path, out var writer))
            {
                try
                {
                    writer = new StreamWriter(path, false) { AutoFlush = true };
                }
                catch(Exception e) when(e is IOException || e is UnauthorizedAccessException)
                {
                    // an error in the monitor instead of a crash
                    throw new RecoverableException(e.Message);
                }
                writers[path] = writer;
            }
            return writer;
        }

        public static string[] ReadLines(string path)
        {
            try
            {
                return File.ReadAllLines(path);
            }
            catch(Exception e) when(e is IOException || e is UnauthorizedAccessException)
            {
                throw new RecoverableException(e.Message);
            }
        }

        private class UartLine
        {
            public StreamWriter Writer;
            public StringBuilder Text = new StringBuilder();
        }

        private static readonly Dictionary<IUART, UartLine> uartLines = new Dictionary<IUART, UartLine>();
        private static readonly Dictionary<ICAN, List<Tuple<ulong, ulong>>> holds = new Dictionary<ICAN, List<Tuple<ulong, ulong>>>();
        private static readonly Dictionary<string, StreamWriter> writers = new Dictionary<string, StreamWriter>();

        private const long CsrOffset = 0x74;
        private const uint Rmvf = 1u << 24;
        private const uint Pinrstf = 1u << 26;
        private const uint Iwdgrstf = 1u << 29;
        private const uint PowerOnFlags = 0x0E000000; // BORRSTF, PINRSTF, PORRSTF
    }

    // Inputs at virtual times, one per line, sorted or not:
    //   <us> can <1|2> <id hex> <s|x><d|r> <data hex, or ->
    //   <us> pin <port letter> <pin> <0|1>      gpioPort<letter> input level
    //   <us> adc <channel> <code>               adc1 conversions from now on
    //   <us> mark <label>                       logs the time
    //   <us> read <label> <address>             logs the 32-bit value there
    //   <us> write <address> <value>            32-bit sysbus write
    //   <us> reset                              reset from outside, like the reset pin
    // Each event that ran is logged as "<us> <the line>" (reads with their value).
    // ADC inputs and pin levels are the outside world, so after a chip reset
    // (which clears them in the models) the latest ones are applied again.
    public class Timeline
    {
        public Timeline(IMachine machine, string path, StreamWriter log)
        {
            this.machine = machine;
            this.log = log;
            var lineNumber = 0;
            foreach(var raw in VirtualTimeRecorders.ReadLines(path))
            {
                lineNumber++;
                var text = raw.Trim();
                if(text == "" || text.StartsWith("#"))
                {
                    continue;
                }
                var parts = text.Split(new[] { ' ' }, 2);
                if(parts.Length < 2 || !ulong.TryParse(parts[0], out var at))
                {
                    throw new RecoverableException($"{path}:{lineNumber}: can't parse '{raw}'");
                }
                events.Add(new Event { AtUs = at, Text = parts[1], Order = events.Count });
            }
            events = events.OrderBy(e => e.AtUs).ThenBy(e => e.Order).ToList();
            machine.MachineReset += _ =>
            {
                foreach(var input in inputs.Values.ToList())
                {
                    Run(input, VirtualTimeRecorders.Now(machine), false);
                }
            };
        }

        public void Start()
        {
            ScheduleNext();
        }

        private void ScheduleNext()
        {
            if(next >= events.Count)
            {
                return;
            }
            var nowNs = machine.ElapsedVirtualTime.TimeElapsed.Ticks;
            var targetNs = events[next].AtUs * 1000;
            var delay = targetNs > nowNs ? targetNs - nowNs : 1;
            machine.ScheduleAction(TimeInterval.FromNanoseconds(delay), _ => RunDue(), "timeline");
        }

        private void RunDue()
        {
            var now = VirtualTimeRecorders.Now(machine);
            while(next < events.Count && events[next].AtUs <= now)
            {
                var e = events[next++];
                try
                {
                    Run(e.Text, now, true);
                }
                catch(Exception ex)
                {
                    log.WriteLine($"{now} error {e.Text}: {ex.Message}");
                }
            }
            ScheduleNext();
        }

        private void Run(string text, ulong now, bool logIt)
        {
            var a = text.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if(a[0] == "adc" || a[0] == "pin")
            {
                inputs[a[0] + " " + a[1] + (a[0] == "pin" ? " " + a[2] : "")] = text;
            }
            switch(a[0])
            {
            case "can":
            {
                var can = Get<ICAN>("sysbus.can" + a[1]);
                var id = uint.Parse(a[2], NumberStyles.HexNumber);
                var data = a.Length > 4 ? VirtualTimeRecorders.Hex(a[4]) : new byte[0];
                can.OnFrameReceived(new CANMessageFrame(id, data, a[3][0] == 'x', a[3][1] == 'r'));
                break;
            }
            case "pin":
                Get<IGPIOReceiver>("sysbus.gpioPort" + a[1]).OnGPIO(int.Parse(a[2]), a[3] == "1");
                break;
            case "adc":
            {
                var adc = Get<IPeripheral>("sysbus.adc1");
                var feed = adc.GetType().GetMethod("FeedSample", new[] { typeof(uint), typeof(uint), typeof(int) });
                feed.Invoke(adc, new object[] { uint.Parse(a[2]), uint.Parse(a[1]), -1 });
                break;
            }
            case "mark":
                break;
            case "read":
            {
                var value = machine.SystemBus.ReadDoubleWord(Convert.ToUInt64(a[2], 16));
                log.WriteLine($"{now} {text} {value:X8}");
                return;
            }
            case "write":
                machine.SystemBus.WriteDoubleWord(Convert.ToUInt64(a[1], 16), Convert.ToUInt32(a[2], 16));
                break;
            case "reset":
                machine.RequestReset();
                break;
            default:
                throw new RecoverableException($"unknown event '{text}'");
            }
            if(logIt)
            {
                log.WriteLine($"{now} {text}");
            }
        }

        private T Get<T>(string name) where T : class, IPeripheral
        {
            if(!machine.TryGetByName<T>(name, out var peripheral))
            {
                throw new RecoverableException($"{name} not found");
            }
            return peripheral;
        }

        private class Event
        {
            public ulong AtUs;
            public string Text;
            public int Order;
        }

        private int next;
        private List<Event> events = new List<Event>();
        private readonly Dictionary<string, string> inputs = new Dictionary<string, string>();

        private readonly IMachine machine;
        private readonly StreamWriter log;
    }
}
