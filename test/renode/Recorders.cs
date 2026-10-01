//
// Monitor commands that write what the firmware does to files, stamped with
// virtual time in microseconds (Renode's own log only has host time), plus a
// way to make a firmware function fail on purpose.
//
//   can1 RecordFrames @can.txt                 <us> <id hex> <data hex>
//   uart4 RecordLines @uart.txt                <us> <line>
//   uart4 FlushLine                            writes a line still missing its "\n"
//   sysbus RecordByteChanges 0x2000... @f.txt  <us> <new value>, on every change
//   machine WriteMarker @marks.txt "x"         <us> x
//   cpu RecordWordAt <pc> <address> @f.txt     <us> <32-bit value at address>, when pc runs
//   cpu ReturnBetween <addr> <r0> <from us> <to us>
//       the function at <addr> returns <r0> without running, inside the window
//

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CAN;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Testing
{
    public static class VirtualTimeRecorders
    {
        public static void RecordFrames(this ICAN can, string path)
        {
            var machine = MachineOf(can);
            var writer = new StreamWriter(path, false) { AutoFlush = true };
            can.FrameSent += frame =>
            {
                var data = string.Concat(frame.Data.Select(b => b.ToString("X2")));
                writer.WriteLine($"{Now(machine)} {frame.Id:X3} {data}");
            };
        }

        public static void RecordLines(this IUART uart, string path)
        {
            var machine = MachineOf(uart);
            var line = new UartLine { Writer = new StreamWriter(path, false) { AutoFlush = true } };
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

        public static void RecordByteChanges(this IBusController bus, ulong address, string path)
        {
            var machine = bus.Machine;
            var writer = new StreamWriter(path, false) { AutoFlush = true };
            ulong? last = null;
            bus.AddWatchpointHook(address, SysbusAccessWidth.Byte, Access.Write, (cpu, a, width, value) =>
            {
                value &= 0xFF;
                if(value != last)
                {
                    last = value;
                    writer.WriteLine($"{Now(machine)} {value}");
                }
            });
        }

        public static void WriteMarker(this IMachine machine, string path, string label)
        {
            File.AppendAllText(path, $"{Now(machine)} {label}\n");
        }

        // Some registers only read back in certain modes (CAN BTR only in init
        // mode), so read them when the firmware reaches a given function
        public static void RecordWordAt(this ICpuSupportingGdb cpu, ulong pc, ulong address, string path)
        {
            var machine = MachineOf(cpu);
            cpu.AddHook(pc & ~1UL, (c, _) =>
            {
                File.AppendAllText(path, $"{Now(machine)} {machine.SystemBus.ReadDoubleWord(address):X8}\n");
            });
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

        private static IMachine MachineOf(IPeripheral peripheral)
        {
            if(!EmulationManager.Instance.CurrentEmulation.TryGetMachineForPeripheral(peripheral, out var machine))
            {
                throw new RecoverableException("Peripheral is not attached to a machine");
            }
            return machine;
        }

        private static ulong Now(IMachine machine)
        {
            return (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
        }

        private class UartLine
        {
            public StreamWriter Writer;
            public StringBuilder Text = new StringBuilder();
        }

        private static readonly Dictionary<IUART, UartLine> uartLines = new Dictionary<IUART, UartLine>();
    }
}
