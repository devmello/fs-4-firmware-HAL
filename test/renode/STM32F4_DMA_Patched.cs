//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
// STM32F4 DMA controller, written for these tests after Renode 1.17's STM32DMA
// (renode-infrastructure cd4b002a), which can't run the VCU's UART5 receive
// ring: it has no circular mode, no half transfer flag or interrupt, and drops
// the channel selection. Changes from that model:
// - Circular mode: NDTR reloads when it reaches 0 and the stream keeps going.
// - Half transfer and transfer complete flags (LISR/HISR), cleared through
//   LIFCR/HIFCR, with their interrupts (HTIE, TCIE, TEIE, DMEIE, FEIE).
// - Every stream register reads back what was written (CHSEL, PL, CIRC, the
//   interrupt enables, FCR), so tests can check the setup.
// - Peripheral requests are numbered stream * 8 + channel (RM0390 table 28),
//   and a stream only takes a request on the channel its CHSEL selects, like
//   the chip. UART5_RX is DMA1 stream 0 channel 4, so request 4.
// - A non-circular stream turns EN off when NDTR reaches 0.
// Peripheral-to-memory transfers move one data unit per request. Memory to
// memory and memory to peripheral still copy everything as soon as EN is set,
// as in the original. No FIFO, bursts, double buffer or priorities.
//

using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
    public sealed class STM32F4_DMA_Patched : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput
    {
        public STM32F4_DMA_Patched(IMachine machine)
        {
            this.machine = machine;
            streams = Enumerable.Range(0, NumberOfStreams).Select(i => new Stream(this, i)).ToArray();
            Connections = Enumerable.Range(0, NumberOfStreams).ToDictionary(i => i, i => (IGPIO)streams[i].IRQ);
            Reset();
        }

        public void Reset()
        {
            foreach(var stream in streams)
            {
                stream.Reset();
            }
        }

        // number = stream * 8 + channel
        public void OnGPIO(int number, bool value)
        {
            if(!value)
            {
                return;
            }
            var stream = number / 8;
            var channel = number % 8;
            if(stream >= NumberOfStreams)
            {
                this.Log(LogLevel.Warning, "DMA request {0} out of range", number);
                return;
            }
            streams[stream].Request(channel);
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case LowInterruptStatus:
                return StatusOf(0);
            case HighInterruptStatus:
                return StatusOf(4);
            case LowInterruptClear:
            case HighInterruptClear:
                return 0;
            }
            if(TryStream(offset, out var stream, out var register))
            {
                return stream.Read(register);
            }
            this.Log(LogLevel.Warning, "Unhandled read at 0x{0:X}", offset);
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case LowInterruptStatus:
            case HighInterruptStatus:
                return; // read only
            case LowInterruptClear:
                ClearFlags(0, value);
                return;
            case HighInterruptClear:
                ClearFlags(4, value);
                return;
            }
            if(TryStream(offset, out var stream, out var register))
            {
                stream.Write(register, value);
                return;
            }
            this.Log(LogLevel.Warning, "Unhandled write at 0x{0:X}, value 0x{1:X}", offset, value);
        }

        public long Size => 0x400;

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        // Requests that reached an enabled stream on the right channel, and
        // ones that didn't (stream off or another channel selected)
        public ulong TakenRequests { get; private set; }

        public ulong IgnoredRequests { get; private set; }

        private uint StatusOf(int firstStream)
        {
            uint value = 0;
            for(var i = 0; i < 4; i++)
            {
                value |= streams[firstStream + i].Flags << FlagShift[i];
            }
            return value;
        }

        private void ClearFlags(int firstStream, uint value)
        {
            for(var i = 0; i < 4; i++)
            {
                streams[firstStream + i].ClearFlags((value >> FlagShift[i]) & 0x3D);
            }
        }

        private bool TryStream(long offset, out Stream stream, out long register)
        {
            stream = null;
            register = 0;
            if(offset < FirstStreamRegister || offset >= FirstStreamRegister + NumberOfStreams * StreamStep)
            {
                return false;
            }
            stream = streams[(offset - FirstStreamRegister) / StreamStep];
            register = (offset - FirstStreamRegister) % StreamStep;
            return true;
        }

        private readonly IMachine machine;
        private readonly Stream[] streams;

        private static readonly int[] FlagShift = { 0, 6, 16, 22 };

        private const int NumberOfStreams = 8;
        private const long StreamStep = 0x18;
        private const long FirstStreamRegister = 0x10;
        private const long LowInterruptStatus = 0x0;
        private const long HighInterruptStatus = 0x4;
        private const long LowInterruptClear = 0x8;
        private const long HighInterruptClear = 0xC;

        private class Stream
        {
            public Stream(STM32F4_DMA_Patched parent, int id)
            {
                this.parent = parent;
                this.id = id;
            }

            public void Reset()
            {
                control = 0;
                remaining = 0;
                reload = 0;
                peripheralAddress = 0;
                memory0Address = 0;
                memory1Address = 0;
                fifoControl = 0x21;
                Flags = 0;
                transferred = 0;
                IRQ.Unset();
            }

            public uint Read(long register)
            {
                switch(register)
                {
                case CR:
                    return control;
                case NDTR:
                    return remaining;
                case PAR:
                    return peripheralAddress;
                case M0AR:
                    return memory0Address;
                case M1AR:
                    return memory1Address;
                case FCR:
                    // FS reads "empty" (100); there is no FIFO model
                    return (fifoControl & 0x87) | (0x4 << 3);
                }
                return 0;
            }

            public void Write(long register, uint value)
            {
                switch(register)
                {
                case CR:
                    WriteControl(value);
                    break;
                case NDTR:
                    if(Enabled)
                    {
                        parent.Log(LogLevel.Warning, "Stream {0}: NDTR written while enabled, ignored", id);
                        break;
                    }
                    remaining = value & 0xFFFF;
                    break;
                case PAR:
                    peripheralAddress = value;
                    break;
                case M0AR:
                    memory0Address = value;
                    break;
                case M1AR:
                    memory1Address = value;
                    break;
                case FCR:
                    fifoControl = value & 0x87;
                    break;
                }
            }

            public void ClearFlags(uint mask)
            {
                if(mask != 0)
                {
                    Flags &= ~mask;
                    UpdateInterrupt();
                }
            }

            public void Request(int channel)
            {
                if(!Enabled || Channel != channel || Direction != PeripheralToMemory)
                {
                    parent.IgnoredRequests++;
                    parent.Log(LogLevel.Debug, "Stream {0}: request on channel {1} ignored (EN {2}, CHSEL {3})", id, channel, Enabled, Channel);
                    return;
                }
                parent.TakenRequests++;
                TransferUnit();
            }

            public uint Flags { get; private set; }

            public GPIO IRQ { get; } = new GPIO();

            private void WriteControl(uint value)
            {
                var wasEnabled = Enabled;
                control = value & 0x0FFFFFFF;
                if(Enabled && !wasEnabled)
                {
                    reload = remaining;
                    transferred = 0;
                    if(remaining == 0)
                    {
                        parent.Log(LogLevel.Warning, "Stream {0} enabled with NDTR 0", id);
                    }
                    else if(Direction != PeripheralToMemory)
                    {
                        // Same as the original model: copy it all at once
                        while(remaining > 0 && Enabled)
                        {
                            TransferUnit();
                        }
                    }
                }
                UpdateInterrupt();
            }

            private void TransferUnit()
            {
                var size = 1 << (int)((control >> 11) & 0x3); // PSIZE; MSIZE is ignored in direct mode
                var memoryStep = (control & MINC) != 0 ? transferred * (uint)size : 0;
                var peripheralStep = (control & PINC) != 0 ? transferred * (uint)size : 0;
                ulong source, destination;
                if(Direction == MemoryToPeripheral)
                {
                    source = memory0Address + memoryStep;
                    destination = peripheralAddress + peripheralStep;
                }
                else
                {
                    source = peripheralAddress + peripheralStep;
                    destination = memory0Address + memoryStep;
                }
                var bus = parent.machine.SystemBus;
                switch(size)
                {
                case 1:
                    bus.WriteByte(destination, bus.ReadByte(source));
                    break;
                case 2:
                    bus.WriteWord(destination, bus.ReadWord(source));
                    break;
                default:
                    bus.WriteDoubleWord(destination, bus.ReadDoubleWord(source));
                    break;
                }

                transferred++;
                remaining--;
                if(remaining == reload / 2)
                {
                    Flags |= HTIF;
                }
                if(remaining == 0)
                {
                    Flags |= TCIF;
                    transferred = 0;
                    if((control & CIRC) != 0)
                    {
                        remaining = reload;
                    }
                    else
                    {
                        control &= ~EN;
                    }
                }
                UpdateInterrupt();
            }

            private void UpdateInterrupt()
            {
                var enabled = 0u;
                if((control & TCIE) != 0) enabled |= TCIF;
                if((control & HTIE) != 0) enabled |= HTIF;
                if((control & TEIE) != 0) enabled |= TEIF;
                if((control & DMEIE) != 0) enabled |= DMEIF;
                if((fifoControl & FEIE) != 0) enabled |= FEIF;
                IRQ.Set((Flags & enabled) != 0);
            }

            private bool Enabled => (control & EN) != 0;

            private int Channel => (int)((control >> 25) & 0x7);

            private uint Direction => (control >> 6) & 0x3;

            private uint control;
            private uint remaining;
            private uint reload;
            private uint transferred;
            private uint peripheralAddress;
            private uint memory0Address;
            private uint memory1Address;
            private uint fifoControl;

            private readonly STM32F4_DMA_Patched parent;
            private readonly int id;

            private const long CR = 0x0;
            private const long NDTR = 0x4;
            private const long PAR = 0x8;
            private const long M0AR = 0xC;
            private const long M1AR = 0x10;
            private const long FCR = 0x14;

            private const uint EN = 1u << 0;
            private const uint DMEIE = 1u << 1;
            private const uint TEIE = 1u << 2;
            private const uint HTIE = 1u << 3;
            private const uint TCIE = 1u << 4;
            private const uint CIRC = 1u << 8;
            private const uint PINC = 1u << 9;
            private const uint MINC = 1u << 10;
            private const uint FEIE = 1u << 7; // in FCR

            // Flag bits, before shifting into LISR/HISR
            private const uint FEIF = 1u << 0;
            private const uint DMEIF = 1u << 2;
            private const uint TEIF = 1u << 3;
            private const uint HTIF = 1u << 4;
            private const uint TCIF = 1u << 5;

            private const uint PeripheralToMemory = 0;
            private const uint MemoryToPeripheral = 1;
        }
    }
}
