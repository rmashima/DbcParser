using uint8_T = System.Byte;
using uint16_T = System.UInt16;
using int64_T = System.Int64;
using uint64_T = System.UInt64;
using System;
using System.Runtime.InteropServices;
using DbcParserLib.Model;

namespace DbcParserLib
{
    public static class Packer
    {
        /// <summary>
        /// Function to pack a signal into a CAN data message
        /// </summary>
        /// <param name="value">Value to be packed</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <returns>Returns a 64 bit unsigned data message</returns>
        public static uint64_T TxSignalPack(double value, Signal signal)
        {
            int64_T iVal = TxPackApplySignAndScale(value, signal);
            uint64_T bitMask = signal.BitMask();

            // Pack signal
            if (signal.Intel()) // Little endian (Intel)
                return (((uint64_T)iVal & bitMask) << signal.StartBit);
            else // Big endian (Motorola)
                return MirrorMsg(((uint64_T)iVal & bitMask) << GetStartBitLE(signal));
        }

        /// <summary>
        /// Function to pack a signal into a CAN data message
        /// </summary>
        /// <param name="message">A ref to the byte array containing the message</param>
        /// <param name="value">Value to be packed</param>
        /// <param name="signal">Signal containing dbc information</param>
        public static void TxSignalPack(uint8_T[] message, double value, Signal signal)
        {
            int64_T iVal = TxPackApplySignAndScale(value, signal);

            // Pack signal
            if (!signal.Intel())
            {
                Array.Reverse(message);
                WriteBits(message, unchecked((uint64_T)iVal), GetStartBitLE(signal, message.Length), signal.Length);
                Array.Reverse(message);
                return;
            }
            WriteBits(message, unchecked((uint64_T)iVal), signal.StartBit, signal.Length);
        }

        /// <summary>
        /// Function to pack a state (unsigned integer) into a CAN data message
        /// </summary>
        /// <param name="value">Value to be packed</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <returns>Returns a 64 bit unsigned data message</returns>
        public static uint64_T TxStatePack(uint64_T value, Signal signal)
        {
            uint64_T bitMask = signal.BitMask();

            // Apply overflow protection
            value = CLAMP(value, 0UL, bitMask);

            // Pack signal
            if (signal.Intel()) // Little endian (Intel)
                return ((value & bitMask) << signal.StartBit);
            else // Big endian (Motorola)
                return MirrorMsg((value & bitMask) << GetStartBitLE(signal));
        }

        /// <summary>
        /// Function to unpack a signal from a CAN data message
        /// </summary>
        /// <param name="RxMsg64">The 64 bit unsigned data message</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <returns>Returns a double value representing the unpacked signal</returns>
        public static double RxSignalUnpack(uint64_T RxMsg64, Signal signal)
        {
            uint64_T iVal;
            uint64_T bitMask = signal.BitMask();

            // Unpack signal
            if (signal.Intel()) // Little endian (Intel)
                iVal = ((RxMsg64 >> signal.StartBit) & bitMask);
            else // Big endian (Motorola)
                iVal = ((MirrorMsg(RxMsg64) >> GetStartBitLE(signal)) & bitMask);

            return RxUnpackApplySignAndScale(signal, iVal);
        }

        /// <summary>
        /// Function to unpack a signal from a CAN data message
        /// </summary>
        /// <param name="receiveMessage">The message data</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <returns>Returns a double value representing the unpacked signal</returns>
        public static double RxSignalUnpack(uint8_T[] receiveMessage, Signal signal)
        {
            var startBit = signal.StartBit;
            var message = receiveMessage;

            if (!signal.Intel())
            {
                var tempArray = new uint8_T[message.Length];
                Array.Copy(message, tempArray, message.Length);
                Array.Reverse(tempArray);

                message = tempArray;
                startBit = GetStartBitLE(signal, message.Length);
            }

            var iVal = ExtractBits(message, startBit, signal.Length);

            return RxUnpackApplySignAndScale(signal, iVal);
        }

        /// <summary>
        /// Function to unpack a state (unsigned integer) from a CAN data message
        /// </summary>
        /// <param name="RxMsg64">The 64 bit unsigned data message</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <returns>Returns an unsigned integer representing the unpacked state</returns>
        public static uint64_T RxStateUnpack(uint64_T RxMsg64, Signal signal)
        {
            uint64_T iVal;
            uint64_T bitMask = signal.BitMask();

            // Unpack signal
            if (signal.Intel()) // Little endian (Intel)
                iVal = (RxMsg64 >> signal.StartBit) & bitMask;
            else // Big endian (Motorola)
                iVal = (MirrorMsg(RxMsg64) >> GetStartBitLE(signal)) & bitMask;

            return iVal;
        }

        /// <summary>
        /// Function to pack a multiplexed signal into a CAN data message, handling multiplexer awareness
        /// </summary>
        /// <param name="message">A ref to the byte array containing the message</param>
        /// <param name="value">Value to be packed</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <param name="multiplexerValue">The current value of the multiplexer signal</param>
        /// <returns>True if the signal was packed, false if it was skipped due to multiplexer value mismatch</returns>
        public static bool TxMultiplexedSignalPack(uint8_T[] message, double value, Signal signal, int multiplexerValue)
        {
            var multiplexingInfo = signal.MultiplexingInfo();
            
            // If signal is a multiplexor, pack it like a normal signal and return true
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexor)
            {
                TxSignalPack(message, value, signal);
                return true;
            }
            
            // If signal is multiplexed, check if the multiplexer value matches the signal's group
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexed)
            {
                // If the multiplexer value doesn't match the signal's group, skip packing
                if (multiplexingInfo.Group != multiplexerValue)
                {
                    return false;
                }
            }
            
            // Pack the signal normally
            TxSignalPack(message, value, signal);
            return true;
        }

        /// <summary>
        /// Function to pack a multiplexed signal into a CAN data message, handling multiplexer awareness
        /// </summary>
        /// <param name="value">Value to be packed</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <param name="multiplexerValue">The current value of the multiplexer signal</param>
        /// <returns>Returns a 64 bit unsigned data message, or 0 if the signal was skipped due to multiplexer value mismatch</returns>
        public static uint64_T TxMultiplexedSignalPack(double value, Signal signal, int multiplexerValue)
        {
            var multiplexingInfo = signal.MultiplexingInfo();
            
            // If signal is a multiplexor, pack it like a normal signal
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexor)
            {
                return TxSignalPack(value, signal);
            }
            
            // If signal is multiplexed, check if the multiplexer value matches the signal's group
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexed)
            {
                // If the multiplexer value doesn't match the signal's group, return 0
                if (multiplexingInfo.Group != multiplexerValue)
                {
                    return 0;
                }
            }
            
            // Pack the signal normally
            return TxSignalPack(value, signal);
        }

        /// <summary>
        /// Function to unpack a multiplexed signal from a CAN data message, handling multiplexer awareness
        /// </summary>
        /// <param name="receiveMessage">The message data</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <param name="multiplexerValue">The current value of the multiplexer signal</param>
        /// <param name="unpackedValue">Output parameter containing the unpacked value if successful</returns>
        /// <returns>True if the signal was unpacked, false if it was skipped due to multiplexer value mismatch</returns>
        public static bool RxMultiplexedSignalUnpack(uint8_T[] receiveMessage, Signal signal, int multiplexerValue, out double unpackedValue)
        {
            var multiplexingInfo = signal.MultiplexingInfo();
            
            // If signal is a multiplexor, unpack it like a normal signal
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexor)
            {
                unpackedValue = RxSignalUnpack(receiveMessage, signal);
                return true;
            }
            
            // If signal is multiplexed, check if the multiplexer value matches the signal's group
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexed)
            {
                // If the multiplexer value doesn't match the signal's group, skip unpacking
                if (multiplexingInfo.Group != multiplexerValue)
                {
                    unpackedValue = 0;
                    return false;
                }
            }
            
            // Unpack the signal normally
            unpackedValue = RxSignalUnpack(receiveMessage, signal);
            return true;
        }

        /// <summary>
        /// Function to unpack a multiplexed signal from a CAN data message, handling multiplexer awareness
        /// </summary>
        /// <param name="RxMsg64">The 64 bit unsigned data message</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <param name="multiplexerValue">The current value of the multiplexer signal</param>
        /// <param name="unpackedValue">Output parameter containing the unpacked value if successful</returns>
        /// <returns>True if the signal was unpacked, false if it was skipped due to multiplexer value mismatch</returns>
        public static bool RxMultiplexedSignalUnpack(uint64_T RxMsg64, Signal signal, int multiplexerValue, out double unpackedValue)
        {
            var multiplexingInfo = signal.MultiplexingInfo();
            
            // If signal is a multiplexor, unpack it like a normal signal
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexor)
            {
                unpackedValue = RxSignalUnpack(RxMsg64, signal);
                return true;
            }
            
            // If signal is multiplexed, check if the multiplexer value matches the signal's group
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexed)
            {
                // If the multiplexer value doesn't match the signal's group, skip unpacking
                if (multiplexingInfo.Group != multiplexerValue)
                {
                    unpackedValue = 0;
                    return false;
                }
            }
            
            // Unpack the signal normally
            unpackedValue = RxSignalUnpack(RxMsg64, signal);
            return true;
        }

        /// <summary>
        /// Function to pack a state (unsigned integer) into a CAN data message, with multiplexer awareness
        /// </summary>
        /// <param name="value">Value to be packed</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <param name="multiplexerValue">The current value of the multiplexer signal</param>
        /// <returns>Returns a 64 bit unsigned data message, or 0 if the signal was skipped due to multiplexer value mismatch</returns>
        public static uint64_T TxMultiplexedStatePack(uint64_T value, Signal signal, int multiplexerValue)
        {
            var multiplexingInfo = signal.MultiplexingInfo();
            
            // If signal is a multiplexor, pack it like a normal signal
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexor)
            {
                return TxStatePack(value, signal);
            }
            
            // If signal is multiplexed, check if the multiplexer value matches the signal's group
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexed)
            {
                // If the multiplexer value doesn't match the signal's group, return 0
                if (multiplexingInfo.Group != multiplexerValue)
                {
                    return 0;
                }
            }
            
            // Pack the signal normally
            return TxStatePack(value, signal);
        }

        /// <summary>
        /// Function to unpack a state (unsigned integer) from a CAN data message, with multiplexer awareness
        /// </summary>
        /// <param name="RxMsg64">The 64 bit unsigned data message</param>
        /// <param name="signal">Signal containing dbc information</param>
        /// <param name="multiplexerValue">The current value of the multiplexer signal</param>
        /// <param name="unpackedValue">Output parameter containing the unpacked value if successful</returns>
        /// <returns>True if the signal was unpacked, false if it was skipped due to multiplexer value mismatch</returns>
        public static bool RxMultiplexedStateUnpack(uint64_T RxMsg64, Signal signal, int multiplexerValue, out uint64_T unpackedValue)
        {
            var multiplexingInfo = signal.MultiplexingInfo();
            
            // If signal is a multiplexor, unpack it like a normal signal
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexor)
            {
                unpackedValue = RxStateUnpack(RxMsg64, signal);
                return true;
            }
            
            // If signal is multiplexed, check if the multiplexer value matches the signal's group
            if (multiplexingInfo.Role == MultiplexingRole.Multiplexed)
            {
                // If the multiplexer value doesn't match the signal's group, skip unpacking
                if (multiplexingInfo.Group != multiplexerValue)
                {
                    unpackedValue = 0;
                    return false;
                }
            }
            
            // Unpack the signal normally
            unpackedValue = RxStateUnpack(RxMsg64, signal);
            return true;
        }

        private static int64_T TxPackApplySignAndScale(double value, Signal signal)
        {
            int64_T iVal;
            uint64_T bitMask = signal.BitMask();

            // Apply scaling
            var rawValue = (value - signal.Offset) / signal.Factor;
            if (signal.ValueType == DbcValueType.IEEEFloat)
                iVal = FloatConverter.AsInteger((float)rawValue);
            else if (signal.ValueType == DbcValueType.IEEEDouble)
                iVal = DoubleConverter.AsInteger(rawValue);
            else
                iVal = (int64_T)Math.Round(rawValue);

            // Apply overflow protection
            if (signal.ValueType == DbcValueType.Signed)
                iVal = CLAMP(iVal, -(int64_T)(bitMask >> 1) - 1, (int64_T)(bitMask >> 1));
            else if (signal.ValueType == DbcValueType.Unsigned)
                iVal = CLAMP(iVal, 0L, (int64_T)bitMask);

            // Manage sign bit (if signed)
            if (signal.ValueType == DbcValueType.Signed && iVal < 0)
            {
                iVal += (int64_T)(1UL << signal.Length);
            }

            return iVal;
        }

        private static double RxUnpackApplySignAndScale(Signal signal, ulong value)
        {
            switch (signal.ValueType)
            {
                case DbcValueType.Signed:
                    int64_T signedValue;
                    if (signal.Length == 64)
                    {
                        signedValue = unchecked((int64_T)value);
                    }
                    else
                    {
                        signedValue = Convert.ToInt64(value);
                        if ((value >> (signal.Length - 1)) != 0)
                        {
                            signedValue -= (1L << signal.Length);
                        }
                    }
                    return (double)(signedValue * (decimal)signal.Factor + (decimal)signal.Offset);
                case DbcValueType.IEEEFloat:
                    return FloatConverter.AsFloatingPoint((int)value) * signal.Factor + signal.Offset;
                case DbcValueType.IEEEDouble:
                    return DoubleConverter.AsFloatingPoint(unchecked((int64_T)value)) * signal.Factor + signal.Offset;
                default:
                    return (double)(value * (decimal)signal.Factor + (decimal)signal.Offset);
            }            
        }

        private static int64_T CLAMP(int64_T x, int64_T low, int64_T high)
        {
            return Math.Max(low, Math.Min(x, high));
        }

        private static uint64_T CLAMP(uint64_T x, uint64_T low, uint64_T high)
        {
            return Math.Max(low, Math.Min(x, high));
        }

        /// <summary>
        /// Mirror data message. It is used to convert Big endian to Little endian and vice-versa
        /// </summary>
        private static uint64_T MirrorMsg(uint64_T msg)
        {
            uint8_T[] v =
            {
                (uint8_T)msg,
                (uint8_T)(msg >> 8),
                (uint8_T)(msg >> 16),
                (uint8_T)(msg >> 24),
                (uint8_T)(msg >> 32),
                (uint8_T)(msg >> 40),
                (uint8_T)(msg >> 48),
                (uint8_T)(msg >> 56)
            };
            return (((uint64_T)v[0] << 56)
                    | ((uint64_T)v[1] << 48)
                    | ((uint64_T)v[2] << 40)
                    | ((uint64_T)v[3] << 32)
                    | ((uint64_T)v[4] << 24)
                    | ((uint64_T)v[5] << 16)
                    | ((uint64_T)v[6] << 8)
                    | (uint64_T)v[7]);
        }

        /// <summary>
        /// Get start bit Little Endian
        /// </summary>
        private static uint16_T GetStartBitLE(Signal signal, int messageByteCount = 8)
        {
            uint16_T startByte = (uint16_T)(signal.StartBit / 8);
            return (uint16_T)(8 * messageByteCount - (signal.Length + 8 * startByte + (8 * (startByte + 1) - (signal.StartBit + 1)) % 8));
        }

        private static void WriteBits(uint8_T[] data, uint64_T value, int startBit, int length)
        {
            for (int bitIndex = 0; bitIndex < length; bitIndex++)
            {
                int bitPosition = startBit + bitIndex;
                int byteIndex = bitPosition / 8;
                int bitInBytePosition = bitPosition % 8;

                // Extract the bit from the signal value
                uint64_T bitValue = (value >> bitIndex) & 1;

                // Set the bit in the message
                data[byteIndex] |= (byte)(bitValue << bitInBytePosition);
            }
        }

        private static uint64_T ExtractBits(uint8_T[] data, int startBit, int length)
        {
            uint64_T result = 0;
            int bitIndex = 0;

            for (int bitPos = startBit; bitPos < startBit + length; bitPos++)
            {
                int bytePos = bitPos / 8;
                int bitInByte = bitPos % 8;

                if (bytePos >= data.Length)
                    break;

                bool bit = (data[bytePos] & (1 << bitInByte)) != 0;
                if (bit)
                {
                    result |= 1UL << bitIndex;
                }

                bitIndex++;
            }

            return result;
        }
    }

    [StructLayout(LayoutKind.Explicit)]
    public class FloatConverter
    {
        [FieldOffset(0)] public int Integer;
        [FieldOffset(0)] public float Float;

        public static int AsInteger(float value)
        {
            return new FloatConverter() { Float = value }.Integer;
        }

        public static float AsFloatingPoint(int value)
        {
            return new FloatConverter() { Integer = value }.Float;
        }
    }

    [StructLayout(LayoutKind.Explicit)]
    public class DoubleConverter
    {
        [FieldOffset(0)] public int64_T Integer;
        [FieldOffset(0)] public double Float;

        public static int64_T AsInteger(double value)
        {
            return new DoubleConverter() { Float = value }.Integer;
        }

        public static double AsFloatingPoint(int64_T value)
        {
            return new DoubleConverter() { Integer = value }.Float;
        }
    }

    /// <summary>
    /// Helper class to handle multiplexed message packing and unpacking
    /// </summary>
    public static class MultiplexedMessage
    {
        /// <summary>
        /// Pack all signals for a message, handling multiplexer signals automatically
        /// </summary>
        /// <param name="message">The message containing signals</param>
        /// <param name="values">A dictionary of signal name to value mappings</param>
        /// <returns>A byte array representing the packed message</returns>
        public static uint8_T[] PackMessage(Message message, Dictionary<string, double> values)
        {
            // Create byte array for the message data
            var data = new uint8_T[message.DLC];
            
            // First find and pack the multiplexor signal
            var multiplexorSignal = message.Signals.FirstOrDefault(s => s.MultiplexingInfo().Role == MultiplexingRole.Multiplexor);
            int multiplexerValue = 0;
            
            if (multiplexorSignal != null && values.TryGetValue(multiplexorSignal.Name, out var muxValue))
            {
                // Pack the multiplexor signal first
                Packer.TxSignalPack(data, muxValue, multiplexorSignal);
                multiplexerValue = (int)muxValue;
            }
            
            // Then pack all other signals according to the multiplexer value
            foreach (var signal in message.Signals)
            {
                // Skip the multiplexor as it's already packed
                if (signal == multiplexorSignal)
                    continue;
                
                if (values.TryGetValue(signal.Name, out var signalValue))
                {
                    Packer.TxMultiplexedSignalPack(data, signalValue, signal, multiplexerValue);
                }
            }
            
            return data;
        }
        
        /// <summary>
        /// Unpack all signals from a message, handling multiplexer signals automatically
        /// </summary>
        /// <param name="message">The message definition</param>
        /// <param name="data">The received byte array</param>
        /// <returns>A dictionary of signal name to unpacked value mappings</returns>
        public static Dictionary<string, double> UnpackMessage(Message message, uint8_T[] data)
        {
            var result = new Dictionary<string, double>();
            
            // First find and unpack the multiplexor signal
            var multiplexorSignal = message.Signals.FirstOrDefault(s => s.MultiplexingInfo().Role == MultiplexingRole.Multiplexor);
            int multiplexerValue = 0;
            
            if (multiplexorSignal != null)
            {
                // Unpack the multiplexor signal first
                var muxValue = Packer.RxSignalUnpack(data, multiplexorSignal);
                result[multiplexorSignal.Name] = muxValue;
                multiplexerValue = (int)muxValue;
            }
            
            // Then unpack all other signals according to the multiplexer value
            foreach (var signal in message.Signals)
            {
                // Skip the multiplexor as it's already unpacked
                if (signal == multiplexorSignal)
                    continue;
                
                if (Packer.RxMultiplexedSignalUnpack(data, signal, multiplexerValue, out var unpackedValue))
                {
                    result[signal.Name] = unpackedValue;
                }
            }
            
            return result;
        }
        
        /// <summary>
        /// Pack a message with a specific multiplexer value
        /// </summary>
        /// <param name="message">The message containing signals</param>
        /// <param name="multiplexerValue">The multiplexer value to use</param>
        /// <param name="values">A dictionary of signal name to value mappings</param>
        /// <returns>A byte array representing the packed message</returns>
        public static uint8_T[] PackMessageWithMultiplexer(Message message, int multiplexerValue, Dictionary<string, double> values)
        {
            // Create byte array for the message data
            var data = new uint8_T[message.DLC];
            
            // First find and pack the multiplexor signal
            var multiplexorSignal = message.Signals.FirstOrDefault(s => s.MultiplexingInfo().Role == MultiplexingRole.Multiplexor);
            
            if (multiplexorSignal != null)
            {
                // Pack the multiplexor signal with the provided value
                Packer.TxSignalPack(data, multiplexerValue, multiplexorSignal);
            }
            else
            {
                // No multiplexor found, but we'll still try to pack with the provided multiplexer value
                // This is useful for cases where the multiplexor signal isn't explicitly marked in the DBC
            }
            
            // Then pack all other signals according to the multiplexer value
            foreach (var signal in message.Signals)
            {
                // Skip the multiplexor as it's already packed
                if (signal == multiplexorSignal)
                    continue;
                
                if (values.TryGetValue(signal.Name, out var signalValue))
                {
                    Packer.TxMultiplexedSignalPack(data, signalValue, signal, multiplexerValue);
                }
            }
            
            return data;
        }
    }
}
