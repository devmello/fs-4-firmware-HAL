//
// A VectorNav VN-200 on the far end of a UART, for the VCU tests.
//
//   uart5 AttachFakeVN200 0x40005000 @vn200.txt
//
// It powers up like the sensor: async ASCII output on (VNINS at 40 Hz, the
// factory default for registers 6 and 7), binary outputs off. It answers the
// ASCII commands the VCU sends (ICD 1.1.2, 1.3): a good command is echoed back
// with its checksum, a bad one gets $VNERR,<code> (ICD table 1.6, sent as two
// hex digits, the way the VCU driver reads it). Supported:
//   $VNASY,0|1                pause / resume async output (ICD 1.3.10)
//   $VNWRG,06,<type>          ASCII async output type, 0 = off
//   $VNWRG,07,<Hz>            ASCII async output rate
//   $VNWRG,75|76|77,<mode>,<divisor>,<groups>,<type words...>
//                             binary outputs 1-3 (ICD 3.2.7-3.2.9, A.1.1)
//   $VNRRG,<reg>              reads the same registers back, and 01 (model)
//   $VNWNV                    accepted, nothing saved
// Binary messages can carry IMU Accel, IMU AngularRate, Attitude Ypr, INS
// PosLla and INS VelBody (the fields the VCU uses); asking for any other field
// gets error 07. A register write that would need more than the baud rate can
// carry is refused with error 0C (insufficient baud rate), as the sensor does.
// The VCU is on serial port 2, so a binary output only comes here when bit 1
// of its AsyncMode is set.
//
// Output goes into the UART, which delivers one byte per character time.
// Messages are queued like the sensor's 2 KB output buffer; whatever doesn't
// fit is dropped and logged. Nothing goes in while the UART is disabled.
//
// Binary message n of an output is generated at phaseUs + n * period, where
// period = divisor / 800 Hz, and its values depend only on n (see Values), so
// the test can work out what every message contained.
//
// The timeline drives it with "imu <command>", see Command().
// Log lines are "<us> <event> ...", among them:
//   rx <line>                       a command from the VCU
//   tx <done us> <line>             a reply, done = when its last byte arrives
//   pkt <output> <n> <bytes> <done us> [corrupt]   binary message n
//

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals.UART;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Testing
{
    public static class FakeVN200Extensions
    {
        public static void AttachFakeVN200(this IUART uart, ulong uartBase, string logPath, ulong phaseUs = 7875)
        {
            if(!EmulationManager.Instance.CurrentEmulation.TryGetMachineForPeripheral(uart, out var machine))
            {
                throw new RecoverableException("UART is not attached to a machine");
            }
            Sensors[uart] = new FakeVN200(machine, uart, uartBase, logPath, phaseUs);
        }

        public static void FakeVN200Command(this IUART uart, string command)
        {
            Of(uart).Command(command);
        }

        public static FakeVN200 Of(IUART uart)
        {
            if(!Sensors.TryGetValue(uart, out var sensor))
            {
                throw new RecoverableException("No fake VN-200 on this UART");
            }
            return sensor;
        }

        private static readonly Dictionary<IUART, FakeVN200> Sensors = new Dictionary<IUART, FakeVN200>();
    }

    public class FakeVN200
    {
        public FakeVN200(IMachine machine, IUART uart, ulong uartBase, string logPath, ulong phaseUs)
        {
            this.machine = machine;
            this.uart = uart;
            this.uartBase = uartBase;
            this.phaseUs = phaseUs;
            try
            {
                log = new StreamWriter(logPath, false) { AutoFlush = true };
            }
            catch(Exception e) when(e is IOException || e is UnauthorizedAccessException)
            {
                throw new RecoverableException(e.Message);
            }
            uart.CharReceived += OnByteFromVcu;
            // A reset of the VCU empties the UART's queue: whatever was still on
            // its way is gone, and the line is free
            machine.MachineReset += _ => lineFreeNs = 0;
            PowerOn();
        }

        // Timeline commands:
        //   off            power off: no output, input ignored
        //   on             power on with factory settings
        //   reset          off and on again
        //   mute / unmute  stop / restart all output, settings kept (TX wire cut)
        //   corrupt <n>    flip a bit in the next n binary messages (CRC fails)
        //   error <hex>    send an asynchronous $VNERR,<code> now
        //   inject <hex>   send these bytes now
        public void Command(string command)
        {
            var args = command.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            Log($"command {command}");
            switch(args[0])
            {
            case "off":
                PowerOff();
                break;
            case "on":
                PowerOn();
                break;
            case "reset":
                PowerOff();
                PowerOn();
                break;
            case "mute":
                muted = true;
                break;
            case "unmute":
                muted = false;
                break;
            case "corrupt":
                corruptCount = int.Parse(args[1]);
                break;
            case "error":
                Error(Convert.ToInt32(args[1], 16), "asynchronous");
                break;
            case "inject":
                if(Send(HexToBytes(args[1]), "inject"))
                {
                    Log($"inject {lineFreeNs / 1000} {args[1]}");
                }
                break;
            default:
                throw new RecoverableException($"Unknown fake VN-200 command: {command}");
            }
        }

        // The values in binary message n
        public static void Values(long n, out float[] accel, out float[] gyro, out float[] ypr, out double[] lla, out float[] vel)
        {
            accel = new float[]
            {
                (float)(1.25 + 0.01 * (n % 100)),
                (float)(-0.75 - 0.013 * (n % 50)),
                (float)(-9.80665 + 0.0021 * (n % 30)),
            };
            gyro = new float[]
            {
                (float)(0.0175 * ((n % 40) - 20)),
                (float)(-0.25 + 0.003 * (n % 60)),
                (float)(0.5 - 0.0071 * (n % 25)),
            };
            ypr = new float[]
            {
                (float)(-179.5 + 3.61 * (n % 99)),
                (float)(12.25 - 0.5 * (n % 13)),
                (float)(-3.3 + 0.07 * (n % 90)),
            };
            lla = new double[]
            {
                36.99999123 + 1e-6 * n,
                -122.06123456 - 1e-6 * n,
                12.5 + 0.1 * (n % 10),
            };
            vel = new float[]
            {
                (float)(27.75 - 0.25 * (n % 40)),
                (float)(-0.5 + 0.02 * (n % 50)),
                (float)(0.031 * ((n % 20) - 10)),
            };
        }

        private void PowerOn()
        {
            if(powered)
            {
                return;
            }
            powered = true;
            generation++;
            asyncPaused = false;
            asciiType = 22; // VNINS
            asciiRateHz = 40;
            for(var i = 0; i < 3; i++)
            {
                binary[i] = new BinaryOutput();
            }
            rxLine.Clear();
            muted = false;
            Log("power on");
            ScheduleAscii(generation);
        }

        private void PowerOff()
        {
            if(!powered)
            {
                return;
            }
            powered = false;
            generation++;
            Log("power off");
        }

        // Commands are logged even when the sensor is off: the log shows
        // what the VCU sends
        private void OnByteFromVcu(byte b)
        {
            if(b == '$')
            {
                rxLine.Clear();
            }
            if(rxLine.Length > 256)
            {
                rxLine.Clear();
            }
            rxLine.Append((char)b);
            if(b == '\n')
            {
                var line = rxLine.ToString();
                rxLine.Clear();
                Log($"rx {line.TrimEnd('\r', '\n')}");
                if(!powered)
                {
                    return;
                }
                var gen = generation;
                // The sensor answers within a few milliseconds
                machine.ScheduleAction(TimeInterval.FromMicroseconds(ReplyDelayUs), _ =>
                {
                    if(gen == generation)
                    {
                        HandleLine(line);
                    }
                }, "vn200 reply");
            }
        }

        private void HandleLine(string line)
        {
            line = line.TrimEnd('\r', '\n');
            if(!line.StartsWith("$"))
            {
                return;
            }
            var star = line.LastIndexOf('*');
            if(star < 0)
            {
                Error(3, "no checksum");
                return;
            }
            var payload = line.Substring(1, star - 1);
            var sum = line.Substring(star + 1);
            if(sum != "XX" && !ChecksumMatches(payload, sum))
            {
                Error(3, $"checksum {sum}");
                return;
            }

            var fields = payload.Split(',');
            switch(fields[0])
            {
            case "VNASY":
                if(fields.Length != 2 || (fields[1] != "0" && fields[1] != "1"))
                {
                    Error(fields.Length < 2 ? 5 : fields.Length > 2 ? 6 : 7, payload);
                    return;
                }
                asyncPaused = fields[1] == "0";
                Log($"async {(asyncPaused ? "paused" : "on")}");
                Reply(payload);
                if(!asyncPaused)
                {
                    StartBinaryOutputs();
                }
                return;
            case "VNWRG":
                WriteRegister(fields, payload);
                return;
            case "VNRRG":
                ReadRegister(fields);
                return;
            case "VNWNV":
                Reply(payload);
                return;
            default:
                Error(4, fields[0]);
                return;
            }
        }

        private void WriteRegister(string[] fields, string payload)
        {
            if(fields.Length < 3 || !int.TryParse(fields[1], NumberStyles.None, CultureInfo.InvariantCulture, out var register))
            {
                Error(fields.Length < 3 ? 5 : 7, payload);
                return;
            }
            switch(register)
            {
            case 6:
            case 7:
                if(fields.Length != 3)
                {
                    Error(6, payload);
                    return;
                }
                if(!uint.TryParse(fields[2], NumberStyles.None, CultureInfo.InvariantCulture, out var value))
                {
                    Error(7, payload);
                    return;
                }
                var newType = register == 6 ? value : asciiType;
                var newRate = register == 7 ? value : asciiRateHz;
                if(register == 7 && !AsciiRates.Contains(value))
                {
                    Error(7, payload);
                    return;
                }
                if(BytesPerSecond(newType, newRate, binary) > LinkBytesPerSecond)
                {
                    Error(12, payload);
                    return;
                }
                asciiType = newType;
                asciiRateHz = newRate;
                Log($"reg {register} {value}");
                Reply(payload);
                ScheduleAscii(generation);
                return;
            case 75:
            case 76:
            case 77:
                if(!ParseBinaryOutput(fields, out var output, out var code))
                {
                    Error(code, payload);
                    return;
                }
                var trial = binary.ToArray();
                trial[register - 75] = output;
                if(BytesPerSecond(asciiType, asciiRateHz, trial) > LinkBytesPerSecond)
                {
                    Error(12, payload);
                    return;
                }
                binary[register - 75] = output;
                Log($"reg {register} mode {output.AsyncMode} divisor {output.RateDivisor} groups {output.Groups:X2} types {string.Join(" ", output.Types.Select(t => t.ToString("X4")))} size {output.PacketSize}");
                Reply(payload);
                StartBinaryOutputs();
                return;
            default:
                Error(8, payload);
                return;
            }
        }

        private void ReadRegister(string[] fields)
        {
            if(fields.Length != 2 || !int.TryParse(fields[1], NumberStyles.None, CultureInfo.InvariantCulture, out var register))
            {
                Error(fields.Length < 2 ? 5 : 6, string.Join(",", fields));
                return;
            }
            switch(register)
            {
            case 1:
                Reply("VNRRG,01,VN-200T-CR");
                return;
            case 6:
                Reply($"VNRRG,06,{asciiType}");
                return;
            case 7:
                Reply($"VNRRG,07,{asciiRateHz}");
                return;
            case 75:
            case 76:
            case 77:
                var o = binary[register - 75];
                var types = o.Types.Select(t => t.ToString("X4"));
                Reply($"VNRRG,{register},{o.AsyncMode},{o.RateDivisor},{o.Groups:X2}" + string.Concat(types.Select(t => "," + t)));
                return;
            default:
                Error(8, string.Join(",", fields));
                return;
            }
        }

        // <mode>,<divisor>,<groups>,<one type word per group bit>, all but the
        // first two in hex
        private bool ParseBinaryOutput(string[] fields, out BinaryOutput output, out int code)
        {
            output = null;
            code = 0;
            if(fields.Length < 5)
            {
                code = 5;
                return false;
            }
            if(!uint.TryParse(fields[2], NumberStyles.None, CultureInfo.InvariantCulture, out var mode)
               || !uint.TryParse(fields[3], NumberStyles.None, CultureInfo.InvariantCulture, out var divisor)
               || !uint.TryParse(fields[4], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var groups)
               || mode > 3 || divisor > 0xFFFF || groups > 0x7F)
            {
                code = 7;
                return false;
            }
            var groupCount = Enumerable.Range(0, 8).Count(b => (groups & (1u << b)) != 0);
            if(fields.Length - 5 != groupCount)
            {
                code = fields.Length - 5 < groupCount ? 5 : 6;
                return false;
            }
            var types = new List<uint>();
            for(var i = 0; i < groupCount; i++)
            {
                if(!uint.TryParse(fields[5 + i], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var t) || t > 0xFFFF)
                {
                    code = 7;
                    return false;
                }
                types.Add(t);
            }
            output = new BinaryOutput { AsyncMode = mode, RateDivisor = divisor, Groups = groups, Types = types.ToArray() };
            if(groups != 0 && output.Fields() == null)
            {
                code = 7; // a field this fake doesn't produce
                return false;
            }
            return true;
        }

        private void StartBinaryOutputs()
        {
            // Each output runs on its own schedule, restarted on every change
            binaryGeneration++;
            for(var i = 0; i < 3; i++)
            {
                var o = binary[i];
                if(o.AsyncMode == 0 || o.Groups == 0 || o.RateDivisor == 0)
                {
                    continue;
                }
                var periodUs = o.RateDivisor * 1250UL; // 800 Hz base rate
                var now = NowUs();
                // First message strictly after now
                var n = now < phaseUs ? 0 : (long)((now - phaseUs) / periodUs) + 1;
                ScheduleBinary(i, periodUs, n, binaryGeneration, generation);
            }
        }

        private void ScheduleBinary(int index, ulong periodUs, long n, long binaryGen, long gen)
        {
            At(phaseUs + (ulong)n * periodUs, () =>
            {
                if(binaryGen != binaryGeneration || gen != generation)
                {
                    return;
                }
                EmitBinary(index, n);
                ScheduleBinary(index, periodUs, n + 1, binaryGen, gen);
            });
        }

        private void EmitBinary(int index, long n)
        {
            var o = binary[index];
            if(asyncPaused || (o.AsyncMode & 2) == 0)
            {
                return; // port 2 only gets it with AsyncMode bit 1
            }
            var packet = o.Build(n);
            var corrupt = corruptCount > 0;
            if(corrupt)
            {
                // An exponent bit of accel X (byte 11), so a message decoded
                // despite its CRC would show in frame 720
                corruptCount--;
                packet[11] ^= 0x01;
            }
            if(Send(packet, $"bin{index + 1} {n}{(corrupt ? " corrupt" : "")}"))
            {
                // and when its last byte will have reached the UART
                Log($"pkt {index + 1} {n} {packet.Length} {lineFreeNs / 1000}{(corrupt ? " corrupt" : "")}");
            }
        }

        private void ScheduleAscii(long gen)
        {
            asciiGeneration++;
            var myGen = asciiGeneration;
            if(asciiType == 0 || asciiRateHz == 0)
            {
                return;
            }
            var periodUs = 1000000UL / asciiRateHz;
            ScheduleAsciiAt(periodUs, (long)(NowUs() / periodUs) + 1, myGen, gen);
        }

        private void ScheduleAsciiAt(ulong periodUs, long n, long asciiGen, long gen)
        {
            At((ulong)n * periodUs, () =>
            {
                if(asciiGen != asciiGeneration || gen != generation)
                {
                    return;
                }
                if(!asyncPaused)
                {
                    var text = Line(InsLine(n));
                    Send(Encoding.ASCII.GetBytes(text), $"ascii {n}");
                }
                ScheduleAsciiAt(periodUs, n + 1, asciiGen, gen);
            });
        }

        // Shaped like the factory default VNINS output (~130 bytes)
        private static string InsLine(long n)
        {
            var t = 300000.0 + n * 0.025;
            return string.Format(CultureInfo.InvariantCulture,
                "VNINS,{0:000000.000000},{1:0000},0000,{2:+000.000;-000.000},{3:+00.000;-00.000},{4:+000.000;-000.000},+36.99999123,-122.06123456,+00012.500,+000.000,+000.000,+000.000,25.0,03.1,0.25",
                t, 2400 + n / 40000, -179.5 + (n % 99), 12.25, -3.3);
        }

        // Logged as "tx <done us> <line>", done being when its last byte arrives
        private void Reply(string payload)
        {
            var text = Line(payload);
            if(Send(Encoding.ASCII.GetBytes(text), "reply"))
            {
                Log($"tx {lineFreeNs / 1000} {text.TrimEnd('\r', '\n')}");
            }
        }

        private void Error(int code, string what)
        {
            var text = Line($"VNERR,{code:X2}");
            Log($"err {code:X2} {what}");
            if(Send(Encoding.ASCII.GetBytes(text), "error"))
            {
                Log($"tx {lineFreeNs / 1000} {text.TrimEnd('\r', '\n')}");
            }
        }

        // Queues bytes for the UART, like the sensor's output buffer
        private bool Send(byte[] bytes, string what)
        {
            if(!powered || muted)
            {
                return false;
            }
            var now = NowNs();
            var byteNs = ByteNs();
            if(byteNs == 0 || !UartReceiving())
            {
                Log($"drop {bytes.Length} {what} (UART not receiving)");
                return false;
            }
            var lineFree = Math.Max(lineFreeNs, now);
            if((lineFree - now) / byteNs + (ulong)bytes.Length > OutputBufferBytes)
            {
                Log($"drop {bytes.Length} {what} (output buffer full)");
                return false;
            }
            lineFreeNs = lineFree + (ulong)bytes.Length * byteNs;
            foreach(var b in bytes)
            {
                uart.WriteChar(b);
            }
            return true;
        }

        private bool UartReceiving()
        {
            // CR1: UE (13) and RE (2)
            var cr1 = machine.SystemBus.ReadDoubleWord(uartBase + 0xC);
            return (cr1 & 0x2004) == 0x2004;
        }

        // Same rounding as the UART model's character time
        private ulong ByteNs()
        {
            var baud = uart.BaudRate;
            return baud == 0 ? 0 : (ulong)Math.Ceiling(10.0 / baud * 1e9);
        }

        private void At(ulong us, Action action)
        {
            var now = NowNs();
            var target = us * 1000;
            var delay = target > now ? target - now : 1;
            machine.ScheduleAction(TimeInterval.FromNanoseconds(delay), _ => action(), "vn200");
        }

        private ulong NowNs()
        {
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                cpu.SyncTime();
            }
            return machine.ElapsedVirtualTime.TimeElapsed.Ticks;
        }

        private ulong NowUs()
        {
            return NowNs() / 1000;
        }

        private void Log(string text)
        {
            log.WriteLine($"{NowUs()} {text}");
        }

        private static string Line(string payload)
        {
            return $"${payload}*{Xor(payload):X2}\r\n";
        }

        private static byte Xor(string payload)
        {
            byte sum = 0;
            foreach(var c in payload)
            {
                sum ^= (byte)c;
            }
            return sum;
        }

        private static bool ChecksumMatches(string payload, string sum)
        {
            if(sum.Length == 2 && byte.TryParse(sum, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var x))
            {
                return x == Xor(payload);
            }
            if(sum.Length == 4 && ushort.TryParse(sum, NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var c))
            {
                return c == Crc16(Encoding.ASCII.GetBytes(payload), 0, payload.Length);
            }
            return false;
        }

        // CRC-16-CCITT, polynomial 0x1021, initial value 0 (ICD 1.4.3)
        public static ushort Crc16(byte[] data, int start, int length)
        {
            ushort crc = 0;
            for(var i = start; i < start + length; i++)
            {
                crc = (ushort)((byte)(crc >> 8) | (crc << 8));
                crc ^= data[i];
                crc ^= (ushort)((byte)(crc & 0xFF) >> 4);
                crc ^= (ushort)((crc << 8) << 4);
                crc ^= (ushort)(((crc & 0xFF) << 4) << 1);
            }
            return crc;
        }

        private static byte[] HexToBytes(string hex)
        {
            return Enumerable.Range(0, hex.Length / 2).Select(i => Convert.ToByte(hex.Substring(2 * i, 2), 16)).ToArray();
        }

        private static ulong BytesPerSecond(uint type, uint rateHz, BinaryOutput[] outputs)
        {
            ulong total = type != 0 ? (ulong)AsciiBytes * rateHz : 0;
            foreach(var o in outputs)
            {
                if(o.AsyncMode != 0 && o.Groups != 0 && o.RateDivisor != 0)
                {
                    total += (ulong)o.PacketSize * 800 / o.RateDivisor;
                }
            }
            return total;
        }

        private ulong LinkBytesPerSecond => uart.BaudRate == 0 ? 11520UL : uart.BaudRate / 10UL;

        private bool powered;
        private bool muted;
        private bool asyncPaused;
        private uint asciiType;
        private uint asciiRateHz;
        private int corruptCount;
        private long generation;
        private long binaryGeneration;
        private long asciiGeneration;
        private ulong lineFreeNs;

        private readonly BinaryOutput[] binary = new BinaryOutput[3];
        private readonly StringBuilder rxLine = new StringBuilder();
        private readonly IMachine machine;
        private readonly IUART uart;
        private readonly ulong uartBase;
        private readonly ulong phaseUs;
        private readonly StreamWriter log;

        private static readonly uint[] AsciiRates = { 1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200 };

        private const int AsciiBytes = 130;
        private const int OutputBufferBytes = 2048;
        private const ulong ReplyDelayUs = 2000;

        private class BinaryOutput
        {
            public uint AsyncMode;
            public uint RateDivisor;
            public uint Groups;
            public uint[] Types = new uint[0];

            public int PacketSize
            {
                get
                {
                    var fields = Fields();
                    return 2 + 2 * Types.Length + (fields?.Sum(f => f.Size) ?? 0) + 2;
                }
            }

            // Payload fields in packet order (groups by bit, then types by
            // bit), or null if one of them isn't modelled
            public List<Field> Fields()
            {
                var fields = new List<Field>();
                var t = 0;
                for(var g = 0; g < 8; g++)
                {
                    if((Groups & (1u << g)) == 0)
                    {
                        continue;
                    }
                    var types = Types[t++];
                    for(var b = 0; b < 16; b++)
                    {
                        if((types & (1u << b)) == 0)
                        {
                            continue;
                        }
                        var f = Known.FirstOrDefault(k => k.Group == g && k.Bit == b);
                        if(f == null)
                        {
                            return null;
                        }
                        fields.Add(f);
                    }
                }
                return fields;
            }

            public byte[] Build(long n)
            {
                FakeVN200.Values(n, out var accel, out var gyro, out var ypr, out var lla, out var vel);
                var bytes = new List<byte> { 0xFA, (byte)Groups };
                foreach(var t in Types)
                {
                    bytes.Add((byte)t);
                    bytes.Add((byte)(t >> 8));
                }
                foreach(var f in Fields())
                {
                    switch(f.Name)
                    {
                    case "Accel":
                        foreach(var v in accel) bytes.AddRange(BitConverter.GetBytes(v));
                        break;
                    case "AngularRate":
                        foreach(var v in gyro) bytes.AddRange(BitConverter.GetBytes(v));
                        break;
                    case "Ypr":
                        foreach(var v in ypr) bytes.AddRange(BitConverter.GetBytes(v));
                        break;
                    case "PosLla":
                        foreach(var v in lla) bytes.AddRange(BitConverter.GetBytes(v));
                        break;
                    case "VelBody":
                        foreach(var v in vel) bytes.AddRange(BitConverter.GetBytes(v));
                        break;
                    }
                }
                var array = bytes.ToArray();
                var crc = Crc16(array, 1, array.Length - 1);
                return array.Concat(new[] { (byte)(crc >> 8), (byte)crc }).ToArray();
            }

            private static readonly Field[] Known =
            {
                new Field { Group = 2, Bit = 9, Name = "Accel", Size = 12 },
                new Field { Group = 2, Bit = 10, Name = "AngularRate", Size = 12 },
                new Field { Group = 4, Bit = 1, Name = "Ypr", Size = 12 },
                new Field { Group = 5, Bit = 1, Name = "PosLla", Size = 24 },
                new Field { Group = 5, Bit = 3, Name = "VelBody", Size = 12 },
            };
        }

        private class Field
        {
            public int Group;
            public int Bit;
            public string Name;
            public int Size;
        }
    }
}
