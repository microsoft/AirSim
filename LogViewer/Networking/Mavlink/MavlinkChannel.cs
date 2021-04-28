using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Networking.Mavlink
{
    public class MavLinkMessage
    {
        public const byte Mavlink1Stx = 254;
        public const byte Mavlink2Stx = 253;

        public byte Magic { get; set; }
        public byte Length { get; set; }
        public byte IncompatFlags { get; set; } // flags that must be understood
        public byte CompatFlags { get; set; }   // flags that can be ignored if not understood
        public byte SequenceNumber { get; set; }
        public byte SystemId { get; set; }
        public byte ComponentId { get; set; }
        public MAVLink.MAVLINK_MSG_ID MsgId { get; set; }
        public ushort Crc { get; set; }
        public UInt64 Time { get; set; }
        public byte[] Payload { get; set; }
        public byte[] Signature { get; set; }
        public byte ProtocolVersion { get; set; }
        public object TypedPayload { get; set; }

        public override string ToString()
        {
            return Newtonsoft.Json.JsonConvert.SerializeObject(this);
        }

        public byte[] Pack()
        {
            const byte MAVLINK_STX = 254;
            Serialize();
            int length = Payload.Length;
            if (length + 7 > 255)
            {
                throw new Exception("Message is too long.  Must be less than 248 bytes");
            }
            byte[] fullMessage = new byte[length + 8];
            fullMessage[0] = MAVLINK_STX; // magic marker
            fullMessage[1] = (byte)length;
            fullMessage[2] = this.SequenceNumber;
            fullMessage[3] = this.SystemId;
            fullMessage[4] = this.ComponentId;
            fullMessage[5] = (byte)this.MsgId;
            Array.Copy(Payload, 0, fullMessage, 6, length);
            this.Crc = crc_calculate();
            fullMessage[6 + length] = (byte)(this.Crc);
            fullMessage[6 + length + 1] = (byte)(this.Crc >> 8);
            return fullMessage;
        }

        const int MAVLINK_STX_MAVLINK1 = 0xFE;

        public void ReadHeader(BinaryReader reader)
        {
            ulong time = reader.ReadUInt64();
            time = ConvertBigEndian(time);
            byte magic = reader.ReadByte();
            byte len = reader.ReadByte();

            while (true)
            {
                // 253 for Mavlink 2.0.
                if ((magic == 254 || magic == 253) && len <= 255)
                {
                    // looks good.
                    break;
                }
                // shift to next byte looking for valid header
                time = (time << 8);
                time += magic;
                magic = len;
                len = reader.ReadByte();
            }

            Time = time;
            Magic = magic;
            Length = len;
            SequenceNumber = reader.ReadByte();
            SystemId = reader.ReadByte();
            ComponentId = reader.ReadByte();
            if (Magic == MAVLINK_STX_MAVLINK1)
            {
                MsgId = (MAVLink.MAVLINK_MSG_ID)reader.ReadByte();
            }
            else
            {
                // 24 bits
                int a = reader.ReadByte();
                int b = reader.ReadByte();
                int c = reader.ReadByte();
                MsgId = (MAVLink.MAVLINK_MSG_ID)(a + (b << 8) + (c << 16));
            }
        }

        private ulong ConvertBigEndian(ulong v)
        {
            ulong result = 0;
            ulong shift = v;
            for (int i = 0; i < 8; i++)
            {
                ulong low = (shift & 0xff);
                result = (result << 8) + low;
                shift >>= 8;
            }
            return result;
        }

        public bool IsValidCrc(byte[] msg, int len)
        {
            ushort crc = crc_calculate(msg, len);
            return crc == this.Crc;
        }

        public ushort crc_calculate()
        {
            return crc_calculate(this.Payload, this.Payload.Length);
        }

        private ushort crc_calculate(byte[] buffer, int len)
        {
            int msgid = (int)this.MsgId;
            byte crc_extra = 0;
            if (msgid < MAVLink.MAVLINK_MESSAGE_CRCS.Length)
            {
                crc_extra = MAVLink.MAVLINK_MESSAGE_CRCS[msgid];
            }
            ushort crcTmp = 0xffff; // X25_INIT_CRC;
            // Start with the MAVLINK_CORE_HEADER_LEN bytes.
            crcTmp = crc_accumulate((byte)this.Length, crcTmp);
            if (this.Magic == MavLinkMessage.Mavlink2Stx)
            {
                crcTmp = crc_accumulate((byte)this.IncompatFlags, crcTmp);
                crcTmp = crc_accumulate((byte)this.CompatFlags, crcTmp);
            }
            crcTmp = crc_accumulate((byte)this.SequenceNumber, crcTmp);
            crcTmp = crc_accumulate((byte)this.SystemId, crcTmp);
            crcTmp = crc_accumulate((byte)this.ComponentId, crcTmp);

            crcTmp = crc_accumulate((byte)this.MsgId, crcTmp);
            if (this.Magic == MavLinkMessage.Mavlink2Stx)
            {
                byte b = (byte)((uint)this.MsgId >> 8);
                crcTmp = crc_accumulate(b, crcTmp);
                b = (byte)((uint)this.MsgId >> 16);
                crcTmp = crc_accumulate(b, crcTmp);
            }
            crcTmp = crc_accumulate(buffer, len, crcTmp);
            return crc_accumulate(crc_extra, crcTmp);
        }

        static ushort crc_accumulate(byte[] msg, int len, ushort crcTmp)
        {
            for (int i = 0; i < len; i++)
            {
                crcTmp = crc_accumulate(msg[i], crcTmp);
            }
            return crcTmp;
        }

        static ushort crc_accumulate(byte data, ushort crcAccum)
        {
            /*Accumulate one byte of data into the CRC*/
            byte tmp;
            tmp = (byte)(data ^ (byte)(crcAccum & 0xff));
            tmp ^= (byte)(tmp << 4);
            return (ushort)((crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
        }

        public static Type GetMavlinkType(uint msgid)
        {
            if (msgid < MAVLink.MAVLINK_MESSAGE_INFO.Length)
            {
                return MAVLink.MAVLINK_MESSAGE_INFO[msgid];
            }
            return null;
        }

        public void Deserialize()
        {
            GCHandle handle = GCHandle.Alloc(this.Payload, GCHandleType.Pinned);
            IntPtr ptr = handle.AddrOfPinnedObject();
            var msgType = GetMavlinkType((uint)this.MsgId);
            if (msgType != null)
            {
                object typed = Marshal.PtrToStructure(ptr, msgType);

                this.TypedPayload = typed;

                if (typed != null && typed.GetType() == typeof(MAVLink.mavlink_statustext_t))
                {
                    // convert the byte[] text to something readable.
                    MAVLink.mavlink_statustext_t s = (MAVLink.mavlink_statustext_t)typed;
                    mavlink_statustext_t2 t2 = new mavlink_statustext_t2(s);
                    this.TypedPayload = t2;
                }
            }
        }

        internal void Serialize()
        {
            if (this.TypedPayload != null && this.Payload == null)
            {
                int size = Marshal.SizeOf(this.TypedPayload);
                IntPtr ptr = Marshal.AllocCoTaskMem(size);
                Marshal.StructureToPtr(this.TypedPayload, ptr, false);
                byte[] block = new byte[size];
                Marshal.Copy(ptr, block, 0, size);
                Marshal.FreeCoTaskMem(ptr);
                this.Payload = block;
                this.Length = (byte)block.Length;
            }
        }


    }

    public class MavlinkChannel
    {
        IPort port;
        byte seqno;

        public MavlinkChannel()
        {
            // plug in our custom mavlink_simulator_telemetry message
            int id = MAVLink.mavlink_telemetry.MessageId;
            Type info = MavLinkMessage.GetMavlinkType((uint)id);
            if (info != null && typeof(MAVLink.mavlink_telemetry) != info)
            {
                throw new Exception("The custom messageid " + id + " is already defined, so we can't use it for mavlink_simulator_telemetry");
            }
            else
            {
                MAVLink.MAVLINK_MESSAGE_INFO[id] = typeof(MAVLink.mavlink_telemetry);
            }
        }

        public void Start(IPort port)
        {
            this.port = port;

            Task.Run(() => ReceiveThread());
        }

        public void Stop()
        {
            IPort p = this.port;
            if (p != null)
            {
                this.port = null;
                p.Close();
            }
        }

        public void SendMessage(MavLinkMessage msg)
        {
            msg.SequenceNumber = seqno++;
            byte[] buffer = msg.Pack();
            if (this.port == null)
            {
                throw new Exception("Please call Start to provide the mavlink Port we are using");
            }
            this.port.Write(buffer, buffer.Length);
        }

        public event EventHandler<MavLinkMessage> MessageReceived;

        void ReceiveThread()
        {
            byte[] buffer = new byte[1];

            const byte MAVLINK_IFLAG_SIGNED = 0x01;
            const int MAVLINK_SIGNATURE_BLOCK_LEN = 13;
            MavLinkMessage msg = null;
            ReadState state = ReadState.Init;
            int payloadPos = 0;
            ushort crc = 0;
            int signaturePos = 0;
            uint msgid = 0;
            int msgIdPos = 0;
            int MaxPayloadLength = 255;
            bool messageComplete = false;
            System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();
            watch.Start();

            try
            {

                while (this.port != null)
                {
                    int len = this.port.Read(buffer, 1);
                    if (len == 1)
                    {
                        byte b = buffer[0];
                        switch (state)
                        {
                            case ReadState.Init:
                                if (b == MavLinkMessage.Mavlink1Stx)
                                {
                                    state = ReadState.GotMagic;
                                    msg = new MavLinkMessage();
                                    msg.Time = (ulong)watch.ElapsedMilliseconds * 1000; // must be in microseconds
                                    msg.Magic = MavLinkMessage.Mavlink1Stx;
                                    payloadPos = 0;
                                    msgIdPos = 0;
                                    msgid = 0;
                                    crc = 0;
                                    messageComplete = false;
                                }
                                else if (b == MavLinkMessage.Mavlink2Stx)
                                {
                                    state = ReadState.GotMagic;
                                    msg = new MavLinkMessage();
                                    msg.Time = (ulong)watch.ElapsedMilliseconds * 1000; // must be in microseconds
                                    msg.Magic = MavLinkMessage.Mavlink2Stx;
                                    payloadPos = 0;
                                    msgIdPos = 0;
                                    msgid = 0;
                                    crc = 0;
                                    messageComplete = false;
                                }
                                break;
                            case ReadState.GotMagic:
                                if (b > MaxPayloadLength)
                                {
                                    state = ReadState.Init;
                                }
                                else
                                {
                                    if (msg.Magic == MavLinkMessage.Mavlink1Stx)
                                    {
                                        msg.IncompatFlags = 0;
                                        msg.CompatFlags = 0;
                                        state = ReadState.GotCompatFlags;
                                    }
                                    else
                                    {
                                        msg.Length = b;
                                        msg.Payload = new byte[msg.Length];
                                        state = ReadState.GotLength;
                                    }
                                }
                                break;
                            case ReadState.GotLength:
                                msg.IncompatFlags = b;
                                state = ReadState.GotIncompatFlags;
                                break;
                            case ReadState.GotIncompatFlags:
                                msg.CompatFlags = b;
                                state = ReadState.GotCompatFlags;
                                break;
                            case ReadState.GotCompatFlags:
                                msg.SequenceNumber = b;
                                state = ReadState.GotSequenceNumber;
                                break;
                            case ReadState.GotSequenceNumber:
                                msg.SystemId = b;
                                state = ReadState.GotSystemId;
                                break;
                            case ReadState.GotSystemId:
                                msg.ComponentId = b;
                                state = ReadState.GotComponentId;
                                break;
                            case ReadState.GotComponentId:
                                if (msg.Magic == MavLinkMessage.Mavlink1Stx)
                                {
                                    msg.MsgId = (MAVLink.MAVLINK_MSG_ID)b;
                                    if (msg.Length == 0)
                                    {
                                        // done!
                                        state = ReadState.GotPayload;
                                    }
                                    else
                                    {
                                        state = ReadState.GotMessageId;
                                    }
                                }
                                else
                                {
                                    // msgid is 24 bits
                                    switch(msgIdPos)
                                    {
                                        case 0:
                                            msgid = b;
                                            break;
                                        case 1:
                                            msgid |= ((uint)b << 8);
                                            break;
                                        case 2:
                                            msgid |= ((uint)b << 16);
                                            msg.MsgId = (MAVLink.MAVLINK_MSG_ID)msgid;
                                            state = ReadState.GotMessageId;
                                            break;
                                    }
                                    msgIdPos++;
                                }
                                break;

                            case ReadState.GotMessageId:
                                // read in the payload.
                                msg.Payload[payloadPos++] = b;
                                if (payloadPos == msg.Length)
                                {
                                    state = ReadState.GotPayload;
                                }
                                break;
                            case ReadState.GotPayload:
                                crc = b;
                                state = ReadState.GotCrc1;
                                break;
                            case ReadState.GotCrc1:
                                crc = (ushort)((b << 8) + crc);
                                // ok, let's see if it's good.
                                msg.Crc = crc;
                                ushort found = crc;
                                if (msg.Payload != null)
                                {
                                    found = msg.crc_calculate();
                                    
                                }
                                if (found != crc && crc != 0)
                                {
                                    // bad crc!!
                                    // reset for next message.
                                    state = ReadState.Init;
                                }
                                else
                                {
                                    if ((msg.IncompatFlags & MAVLINK_IFLAG_SIGNED) == MAVLINK_IFLAG_SIGNED)
                                    {
                                        signaturePos = 0;
                                        msg.Signature = new byte[MAVLINK_SIGNATURE_BLOCK_LEN];
                                        state = ReadState.GetSignature;
                                    }
                                    else
                                    {
                                        messageComplete = true;
                                    }
                                }
                                break;
                            case ReadState.GetSignature:
                                msg.Signature[signaturePos++] = b;
                                if (signaturePos == MAVLINK_SIGNATURE_BLOCK_LEN)
                                {
                                    // todo: check the signature.
                                    messageComplete = true;
                                }
                                break;
                        }

                        if (messageComplete)
                        {
                            // try and deserialize the payload.
                            msg.Deserialize();
                            if (MessageReceived != null)
                            {
                                MessageReceived(this, msg);
                            }
                            // reset for next message.
                            state = ReadState.Init;
                        }
                    }

                }

            }
            catch (Exception)
            {
                // port was closed
            }
        }

        enum ReadState
        {
            Init,
            GotMagic,
            GotIncompatFlags,
            GotCompatFlags,
            GotLength,
            GotSequenceNumber,
            GotSystemId,
            GotComponentId,
            GotMessageId,
            GotPayload,
            GotCrc1,
            GetSignature
        }

    }


    public struct mavlink_statustext_t2
    {
        /// <summary> Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY. </summary>
        public byte severity;
        /// <summary> Status text message, without null termination character </summary>            
        public string text;

        public mavlink_statustext_t2(MAVLink.mavlink_statustext_t s)
        {
            this.severity = s.severity;
            this.text = null;
            if (s.text != null)
            {
                int i = 0;
                int n = s.text.Length;
                for (i = 0; i < n; i++)
                {
                    if (s.text[i] == 0)
                    {
                        break;
                    }
                }
                this.text = Encoding.UTF8.GetString(s.text, 0, i);
            }
        }
    };


}
