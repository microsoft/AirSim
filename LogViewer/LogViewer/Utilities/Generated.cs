using LogViewer.Controls;
using LogViewer.Model;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LogViewer.Utilities
{
    // FMT FMT, 89, BBnNZ, Type,Length,Name,Format,Columns
    public class LogEntryFMT : LogEntry
    {
        public LogEntryFMT()
        {
            Name = "FMT";
        }

        public byte Type;
        public byte Length;
        public string FormatString;
        public string[] Columns;

        public static LogEntryFMT Read(BinaryReader reader)
        {
            LogEntryFMT result = new LogEntryFMT();
            result.Type = reader.ReadByte();
            result.Length = reader.ReadByte();
            result.Name = ReadAsciiString(reader, 4).Trim(NullTerminator);
            result.FormatString = ReadAsciiString(reader, 16).Trim(NullTerminator);
            result.Columns = ReadAsciiString(reader, 64).Trim(NullTerminator).Split(',');
            return result;
        }
        static char[] NullTerminator = new char[] { '\0' };
    }


    // PX4  FMT GPS, 55, QBffLLfffffBHHH, GPSTime,Fix,EPH,EPV,Lat,Lon,Alt,VelN,VelE,VelD,Cog,nSat,SNR,N,J
    // Solo FMT GPS, 45, BIHBcLLeeEefI, Status,TimeMS,Week,NSats,HDop,Lat,Lng,RelAlt,Alt,Spd,GCrs,VZ,T
    class LogEntryGPS
    {
        public UInt64 Timestamp;
        public UInt64 GPSTime;
        public byte Fix;
        public float EPH;
        public float EPV;
        public double Lat;
        public double Lon;
        public float Alt;
        public float VelN;
        public float VelE;
        public float VelD;
        public float Cog;
        public byte nSat;
        public UInt16 SNR;
        public UInt16 N;
        public UInt16 J;

        public LogEntryGPS()
        {
        }

        public LogEntryGPS(LogEntry entry)
        {
            Timestamp = entry.Timestamp;

            if (entry.HasField("TimeUS"))
            {
                // perhaps it is the old format with GWk and TimeUS
                // "QBIHBcLLeeEefB"
                GPSTime = entry.GetField<UInt64>("TimeUS"); // Q
                //byte Status = entry.GetField<byte>("Status"); // B
                //uint GMS = entry.GetField<UInt32>("GMS"); // I
                ushort GWk = entry.GetField<UInt16>("GWk"); // H
                nSat = entry.GetField<byte>("NSats"); // B
                EPH = (float)entry.GetField<double>("HDop"); // c
                Lat = entry.GetField<double>("Lat"); // L
                Lon = entry.GetField<double>("Lng"); // L
                //double RAlt = entry.GetField<double>("RAlt"); // e
                Alt = (float)entry.GetField<double>("Alt"); // e
                //double spd = entry.GetField<double>("Spd"); // E
                //double gcrs = entry.GetField<float>("GCrs"); // e
                //double vz = entry.GetField<float>("VZ"); // f
                //byte u = entry.GetField<byte>("U"); // B
            }
            else if (entry.HasField("GPSTime"))
            {
                GPSTime = entry.GetField<UInt64>("GPSTime");
                Fix = entry.GetField<byte>("Fix");
                EPH = entry.GetField<float>("EPH");
                EPV = entry.GetField<float>("EPV");
                Lat = entry.GetField<double>("Lat"); // L
                Lon = entry.GetField<double>("Lon"); // L
                Alt = entry.GetField<float>("Alt");
                VelN = entry.GetField<float>("VelN");
                VelE = entry.GetField<float>("VelE");
                VelD = entry.GetField<float>("VelD");
                Cog = entry.GetField<float>("Cog");
                nSat = entry.GetField<byte>("nSat");
                SNR = entry.GetField<UInt16>("SNR");
                N = entry.GetField<UInt16>("N");
                J = entry.GetField<UInt16>("J");
            }
            else if (entry.HasField("time_usec"))
            {
                GPSTime = entry.GetField<UInt64>("time_usec");
                Fix = entry.GetField<byte>("fix_type");
                EPH = entry.GetField<float>("eph");
                EPV = entry.GetField<float>("epv");
                Lat = (double)entry.GetField<UInt64>("lat") / 1E7;
                Lon = (double)entry.GetField<UInt64>("lon") / 1E7;
                Alt = entry.GetField<float>("alt");
                VelN = entry.GetField<float>("Vel");
                Cog = entry.GetField<float>("cog");
                nSat = entry.GetField<byte>("satellites_visible");
            }
            else if (entry.HasField("TimeMS"))
            {
                // perhaps it is the old format with GWk and TimeUS
                // "QBIHBcLLeeEefB"
                //byte Status = entry.GetField<byte>("Status"); // B
                GPSTime = entry.GetField<UInt32>("TimeMS") * 1000; // I
                ushort GWk = entry.GetField<UInt16>("Week"); // H
                nSat = entry.GetField<byte>("NSats"); // B
                EPH = (float)entry.GetField<double>("HDop"); // c
                Lat = entry.GetField<double>("Lat"); // L
                Lon = entry.GetField<double>("Lng"); // L
                //double RAlt = entry.GetField<double>("RelAlt"); // e
                Alt = (float)entry.GetField<double>("Alt"); // e
                //double spd = entry.GetField<double>("Spd"); // E
                //double gcrs = entry.GetField<float>("GCrs"); // e
                //double vz = entry.GetField<float>("VZ"); // f
                //byte u = entry.GetField<byte>("U"); // B
            }

            else if (entry.HasField("Time"))
            {
                // DJI converted .dat file format
                GPSTime = entry.GetField<UInt32>("Time");
                nSat = entry.GetField<byte>("Visible:GPS"); 
                EPH = (float)entry.GetField<double>("DOP:H");
                Lat = entry.GetField<double>("Lat");
                Lon = entry.GetField<double>("Long");                
                Alt = (float)entry.GetField<double>("Alt"); // e
            }
        }
    }

    //// FMT ATT, 55, fffffffffffff, qw,qx,qy,qz,Roll,Pitch,Yaw,RollRate,PitchRate,YawRate,GX,GY,GZ
    //class LogEntryATT : LogEntry
    //{
    //    public override string GetName() { return "ATT"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "qw":
    //                return new DataValue() { X = 0, Y = this.qw };
    //            case "qx":
    //                return new DataValue() { X = 0, Y = this.qx };
    //            case "qy":
    //                return new DataValue() { X = 0, Y = this.qy };
    //            case "qz":
    //                return new DataValue() { X = 0, Y = this.qz };
    //            case "Roll":
    //                return new DataValue() { X = 0, Y = this.Roll };
    //            case "Pitch":
    //                return new DataValue() { X = 0, Y = this.Pitch };
    //            case "Yaw":
    //                return new DataValue() { X = 0, Y = this.Yaw };
    //            case "RollRate":
    //                return new DataValue() { X = 0, Y = this.RollRate };
    //            case "PitchRate":
    //                return new DataValue() { X = 0, Y = this.PitchRate };
    //            case "YawRate":
    //                return new DataValue() { X = 0, Y = this.YawRate };
    //            case "GX":
    //                return new DataValue() { X = 0, Y = this.GX };
    //            case "GY":
    //                return new DataValue() { X = 0, Y = this.GY };
    //            case "GZ":
    //                return new DataValue() { X = 0, Y = this.GZ };
    //        }
    //        return null;
    //    }
    //    public float qw;
    //    public float qx;
    //    public float qy;
    //    public float qz;
    //    public float Roll;
    //    public float Pitch;
    //    public float Yaw;
    //    public float RollRate;
    //    public float PitchRate;
    //    public float YawRate;
    //    public float GX;
    //    public float GY;
    //    public float GZ;

    //    public static LogEntryATT Read(BinaryReader reader)
    //    {
    //        LogEntryATT result = new LogEntryATT();
    //        result.qw = reader.ReadSingle();
    //        result.qx = reader.ReadSingle();
    //        result.qy = reader.ReadSingle();
    //        result.qz = reader.ReadSingle();
    //        result.Roll = reader.ReadSingle();
    //        result.Pitch = reader.ReadSingle();
    //        result.Yaw = reader.ReadSingle();
    //        result.RollRate = reader.ReadSingle();
    //        result.PitchRate = reader.ReadSingle();
    //        result.YawRate = reader.ReadSingle();
    //        result.GX = reader.ReadSingle();
    //        result.GY = reader.ReadSingle();
    //        result.GZ = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT ATSP, 35, ffffffff, RollSP,PitchSP,YawSP,ThrustSP,qw,qx,qy,qz
    //class LogEntryATSP : LogEntry
    //{
    //    public override string GetName() { return "ATSP"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RollSP":
    //                return new DataValue() { X = 0, Y = this.RollSP };
    //            case "PitchSP":
    //                return new DataValue() { X = 0, Y = this.PitchSP };
    //            case "YawSP":
    //                return new DataValue() { X = 0, Y = this.YawSP };
    //            case "ThrustSP":
    //                return new DataValue() { X = 0, Y = this.ThrustSP };
    //            case "qw":
    //                return new DataValue() { X = 0, Y = this.qw };
    //            case "qx":
    //                return new DataValue() { X = 0, Y = this.qx };
    //            case "qy":
    //                return new DataValue() { X = 0, Y = this.qy };
    //            case "qz":
    //                return new DataValue() { X = 0, Y = this.qz };
    //        }
    //        return null;
    //    }
    //    public float RollSP;
    //    public float PitchSP;
    //    public float YawSP;
    //    public float ThrustSP;
    //    public float qw;
    //    public float qx;
    //    public float qy;
    //    public float qz;

    //    public static LogEntryATSP Read(BinaryReader reader)
    //    {
    //        LogEntryATSP result = new LogEntryATSP();
    //        result.RollSP = reader.ReadSingle();
    //        result.PitchSP = reader.ReadSingle();
    //        result.YawSP = reader.ReadSingle();
    //        result.ThrustSP = reader.ReadSingle();
    //        result.qw = reader.ReadSingle();
    //        result.qx = reader.ReadSingle();
    //        result.qy = reader.ReadSingle();
    //        result.qz = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT IMU, 51, ffffffffffff, AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM
    //class LogEntryIMU : LogEntry
    //{
    //    public override string GetName() { return "IMU"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "AccX":
    //                return new DataValue() { X = 0, Y = this.AccX };
    //            case "AccY":
    //                return new DataValue() { X = 0, Y = this.AccY };
    //            case "AccZ":
    //                return new DataValue() { X = 0, Y = this.AccZ };
    //            case "GyroX":
    //                return new DataValue() { X = 0, Y = this.GyroX };
    //            case "GyroY":
    //                return new DataValue() { X = 0, Y = this.GyroY };
    //            case "GyroZ":
    //                return new DataValue() { X = 0, Y = this.GyroZ };
    //            case "MagX":
    //                return new DataValue() { X = 0, Y = this.MagX };
    //            case "MagY":
    //                return new DataValue() { X = 0, Y = this.MagY };
    //            case "MagZ":
    //                return new DataValue() { X = 0, Y = this.MagZ };
    //            case "tA":
    //                return new DataValue() { X = 0, Y = this.tA };
    //            case "tG":
    //                return new DataValue() { X = 0, Y = this.tG };
    //            case "tM":
    //                return new DataValue() { X = 0, Y = this.tM };
    //        }
    //        return null;
    //    }
    //    public float AccX;
    //    public float AccY;
    //    public float AccZ;
    //    public float GyroX;
    //    public float GyroY;
    //    public float GyroZ;
    //    public float MagX;
    //    public float MagY;
    //    public float MagZ;
    //    public float tA;
    //    public float tG;
    //    public float tM;

    //    public static LogEntryIMU Read(BinaryReader reader)
    //    {
    //        LogEntryIMU result = new LogEntryIMU();
    //        result.AccX = reader.ReadSingle();
    //        result.AccY = reader.ReadSingle();
    //        result.AccZ = reader.ReadSingle();
    //        result.GyroX = reader.ReadSingle();
    //        result.GyroY = reader.ReadSingle();
    //        result.GyroZ = reader.ReadSingle();
    //        result.MagX = reader.ReadSingle();
    //        result.MagY = reader.ReadSingle();
    //        result.MagZ = reader.ReadSingle();
    //        result.tA = reader.ReadSingle();
    //        result.tG = reader.ReadSingle();
    //        result.tM = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT IMU1, 51, ffffffffffff, AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM
    //class LogEntryIMU1 : LogEntry
    //{
    //    public override string GetName() { return "IMU1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "AccX":
    //                return new DataValue() { X = 0, Y = this.AccX };
    //            case "AccY":
    //                return new DataValue() { X = 0, Y = this.AccY };
    //            case "AccZ":
    //                return new DataValue() { X = 0, Y = this.AccZ };
    //            case "GyroX":
    //                return new DataValue() { X = 0, Y = this.GyroX };
    //            case "GyroY":
    //                return new DataValue() { X = 0, Y = this.GyroY };
    //            case "GyroZ":
    //                return new DataValue() { X = 0, Y = this.GyroZ };
    //            case "MagX":
    //                return new DataValue() { X = 0, Y = this.MagX };
    //            case "MagY":
    //                return new DataValue() { X = 0, Y = this.MagY };
    //            case "MagZ":
    //                return new DataValue() { X = 0, Y = this.MagZ };
    //            case "tA":
    //                return new DataValue() { X = 0, Y = this.tA };
    //            case "tG":
    //                return new DataValue() { X = 0, Y = this.tG };
    //            case "tM":
    //                return new DataValue() { X = 0, Y = this.tM };
    //        }
    //        return null;
    //    }
    //    public float AccX;
    //    public float AccY;
    //    public float AccZ;
    //    public float GyroX;
    //    public float GyroY;
    //    public float GyroZ;
    //    public float MagX;
    //    public float MagY;
    //    public float MagZ;
    //    public float tA;
    //    public float tG;
    //    public float tM;

    //    public static LogEntryIMU1 Read(BinaryReader reader)
    //    {
    //        LogEntryIMU1 result = new LogEntryIMU1();
    //        result.AccX = reader.ReadSingle();
    //        result.AccY = reader.ReadSingle();
    //        result.AccZ = reader.ReadSingle();
    //        result.GyroX = reader.ReadSingle();
    //        result.GyroY = reader.ReadSingle();
    //        result.GyroZ = reader.ReadSingle();
    //        result.MagX = reader.ReadSingle();
    //        result.MagY = reader.ReadSingle();
    //        result.MagZ = reader.ReadSingle();
    //        result.tA = reader.ReadSingle();
    //        result.tG = reader.ReadSingle();
    //        result.tM = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT IMU2, 51, ffffffffffff, AccX,AccY,AccZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,tA,tG,tM
    //class LogEntryIMU2 : LogEntry
    //{
    //    public override string GetName() { return "IMU2"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "AccX":
    //                return new DataValue() { X = 0, Y = this.AccX };
    //            case "AccY":
    //                return new DataValue() { X = 0, Y = this.AccY };
    //            case "AccZ":
    //                return new DataValue() { X = 0, Y = this.AccZ };
    //            case "GyroX":
    //                return new DataValue() { X = 0, Y = this.GyroX };
    //            case "GyroY":
    //                return new DataValue() { X = 0, Y = this.GyroY };
    //            case "GyroZ":
    //                return new DataValue() { X = 0, Y = this.GyroZ };
    //            case "MagX":
    //                return new DataValue() { X = 0, Y = this.MagX };
    //            case "MagY":
    //                return new DataValue() { X = 0, Y = this.MagY };
    //            case "MagZ":
    //                return new DataValue() { X = 0, Y = this.MagZ };
    //            case "tA":
    //                return new DataValue() { X = 0, Y = this.tA };
    //            case "tG":
    //                return new DataValue() { X = 0, Y = this.tG };
    //            case "tM":
    //                return new DataValue() { X = 0, Y = this.tM };
    //        }
    //        return null;
    //    }
    //    public float AccX;
    //    public float AccY;
    //    public float AccZ;
    //    public float GyroX;
    //    public float GyroY;
    //    public float GyroZ;
    //    public float MagX;
    //    public float MagY;
    //    public float MagZ;
    //    public float tA;
    //    public float tG;
    //    public float tM;

    //    public static LogEntryIMU2 Read(BinaryReader reader)
    //    {
    //        LogEntryIMU2 result = new LogEntryIMU2();
    //        result.AccX = reader.ReadSingle();
    //        result.AccY = reader.ReadSingle();
    //        result.AccZ = reader.ReadSingle();
    //        result.GyroX = reader.ReadSingle();
    //        result.GyroY = reader.ReadSingle();
    //        result.GyroZ = reader.ReadSingle();
    //        result.MagX = reader.ReadSingle();
    //        result.MagY = reader.ReadSingle();
    //        result.MagZ = reader.ReadSingle();
    //        result.tA = reader.ReadSingle();
    //        result.tG = reader.ReadSingle();
    //        result.tM = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT SENS, 23, fffff, BaroPres,BaroAlt,BaroTemp,DiffPres,DiffPresFilt
    //class LogEntrySENS : LogEntry
    //{
    //    public override string GetName() { return "SENS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "BaroPres":
    //                return new DataValue() { X = 0, Y = this.BaroPres };
    //            case "BaroAlt":
    //                return new DataValue() { X = 0, Y = this.BaroAlt };
    //            case "BaroTemp":
    //                return new DataValue() { X = 0, Y = this.BaroTemp };
    //            case "DiffPres":
    //                return new DataValue() { X = 0, Y = this.DiffPres };
    //            case "DiffPresFilt":
    //                return new DataValue() { X = 0, Y = this.DiffPresFilt };
    //        }
    //        return null;
    //    }
    //    public float BaroPres;
    //    public float BaroAlt;
    //    public float BaroTemp;
    //    public float DiffPres;
    //    public float DiffPresFilt;

    //    public static LogEntrySENS Read(BinaryReader reader)
    //    {
    //        LogEntrySENS result = new LogEntrySENS();
    //        result.BaroPres = reader.ReadSingle();
    //        result.BaroAlt = reader.ReadSingle();
    //        result.BaroTemp = reader.ReadSingle();
    //        result.DiffPres = reader.ReadSingle();
    //        result.DiffPresFilt = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT AIR1, 23, fffff, BaroPa,BaroAlt,BaroTmp,DiffPres,DiffPresF
    //class LogEntryAIR1 : LogEntry
    //{
    //    public override string GetName() { return "AIR1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "BaroPa":
    //                return new DataValue() { X = 0, Y = this.BaroPa };
    //            case "BaroAlt":
    //                return new DataValue() { X = 0, Y = this.BaroAlt };
    //            case "BaroTmp":
    //                return new DataValue() { X = 0, Y = this.BaroTmp };
    //            case "DiffPres":
    //                return new DataValue() { X = 0, Y = this.DiffPres };
    //            case "DiffPresF":
    //                return new DataValue() { X = 0, Y = this.DiffPresF };
    //        }
    //        return null;
    //    }
    //    public float BaroPa;
    //    public float BaroAlt;
    //    public float BaroTmp;
    //    public float DiffPres;
    //    public float DiffPresF;

    //    public static LogEntryAIR1 Read(BinaryReader reader)
    //    {
    //        LogEntryAIR1 result = new LogEntryAIR1();
    //        result.BaroPa = reader.ReadSingle();
    //        result.BaroAlt = reader.ReadSingle();
    //        result.BaroTmp = reader.ReadSingle();
    //        result.DiffPres = reader.ReadSingle();
    //        result.DiffPresF = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT LPOS, 57, ffffffffLLfBBff, X,Y,Z,Dist,DistR,VX,VY,VZ,RLat,RLon,RAlt,PFlg,GFlg,EPH,EPV
    //class LogEntryLPOS : LogEntry
    //{
    //    public override string GetName() { return "LPOS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "X":
    //                return new DataValue() { X = 0, Y = this.X };
    //            case "Y":
    //                return new DataValue() { X = 0, Y = this.Y };
    //            case "Z":
    //                return new DataValue() { X = 0, Y = this.Z };
    //            case "Dist":
    //                return new DataValue() { X = 0, Y = this.Dist };
    //            case "DistR":
    //                return new DataValue() { X = 0, Y = this.DistR };
    //            case "VX":
    //                return new DataValue() { X = 0, Y = this.VX };
    //            case "VY":
    //                return new DataValue() { X = 0, Y = this.VY };
    //            case "VZ":
    //                return new DataValue() { X = 0, Y = this.VZ };
    //            case "RLat":
    //                return new DataValue() { X = 0, Y = this.RLat };
    //            case "RLon":
    //                return new DataValue() { X = 0, Y = this.RLon };
    //            case "RAlt":
    //                return new DataValue() { X = 0, Y = this.RAlt };
    //            case "PFlg":
    //                return new DataValue() { X = 0, Y = this.PFlg };
    //            case "GFlg":
    //                return new DataValue() { X = 0, Y = this.GFlg };
    //            case "EPH":
    //                return new DataValue() { X = 0, Y = this.EPH };
    //            case "EPV":
    //                return new DataValue() { X = 0, Y = this.EPV };
    //        }
    //        return null;
    //    }
    //    public float X;
    //    public float Y;
    //    public float Z;
    //    public float Dist;
    //    public float DistR;
    //    public float VX;
    //    public float VY;
    //    public float VZ;
    //    public double RLat;
    //    public double RLon;
    //    public float RAlt;
    //    public byte PFlg;
    //    public byte GFlg;
    //    public float EPH;
    //    public float EPV;

    //    public static LogEntryLPOS Read(BinaryReader reader)
    //    {
    //        LogEntryLPOS result = new LogEntryLPOS();
    //        result.X = reader.ReadSingle();
    //        result.Y = reader.ReadSingle();
    //        result.Z = reader.ReadSingle();
    //        result.Dist = reader.ReadSingle();
    //        result.DistR = reader.ReadSingle();
    //        result.VX = reader.ReadSingle();
    //        result.VY = reader.ReadSingle();
    //        result.VZ = reader.ReadSingle();
    //        result.RLat = reader.ReadInt32() / 10000000.0;
    //        result.RLon = reader.ReadInt32() / 10000000.0;
    //        result.RAlt = reader.ReadSingle();
    //        result.PFlg = reader.ReadByte();
    //        result.GFlg = reader.ReadByte();
    //        result.EPH = reader.ReadSingle();
    //        result.EPV = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT LPSP, 43, ffffffffff, X,Y,Z,Yaw,VX,VY,VZ,AX,AY,AZ
    //class LogEntryLPSP : LogEntry
    //{
    //    public override string GetName() { return "LPSP"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "X":
    //                return new DataValue() { X = 0, Y = this.X };
    //            case "Y":
    //                return new DataValue() { X = 0, Y = this.Y };
    //            case "Z":
    //                return new DataValue() { X = 0, Y = this.Z };
    //            case "Yaw":
    //                return new DataValue() { X = 0, Y = this.Yaw };
    //            case "VX":
    //                return new DataValue() { X = 0, Y = this.VX };
    //            case "VY":
    //                return new DataValue() { X = 0, Y = this.VY };
    //            case "VZ":
    //                return new DataValue() { X = 0, Y = this.VZ };
    //            case "AX":
    //                return new DataValue() { X = 0, Y = this.AX };
    //            case "AY":
    //                return new DataValue() { X = 0, Y = this.AY };
    //            case "AZ":
    //                return new DataValue() { X = 0, Y = this.AZ };
    //        }
    //        return null;
    //    }
    //    public float X;
    //    public float Y;
    //    public float Z;
    //    public float Yaw;
    //    public float VX;
    //    public float VY;
    //    public float VZ;
    //    public float AX;
    //    public float AY;
    //    public float AZ;

    //    public static LogEntryLPSP Read(BinaryReader reader)
    //    {
    //        LogEntryLPSP result = new LogEntryLPSP();
    //        result.X = reader.ReadSingle();
    //        result.Y = reader.ReadSingle();
    //        result.Z = reader.ReadSingle();
    //        result.Yaw = reader.ReadSingle();
    //        result.VX = reader.ReadSingle();
    //        result.VY = reader.ReadSingle();
    //        result.VZ = reader.ReadSingle();
    //        result.AX = reader.ReadSingle();
    //        result.AY = reader.ReadSingle();
    //        result.AZ = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT DGPS, 55, QBffLLfffffBHHH, GPSTime,Fix,EPH,EPV,Lat,Lon,Alt,VelN,VelE,VelD,Cog,nSat,SNR,N,J
    //class LogEntryDGPS : LogEntry
    //{
    //    public override string GetName() { return "DGPS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "GPSTime":
    //                return new DataValue() { X = 0, Y = this.GPSTime };
    //            case "Fix":
    //                return new DataValue() { X = 0, Y = this.Fix };
    //            case "EPH":
    //                return new DataValue() { X = 0, Y = this.EPH };
    //            case "EPV":
    //                return new DataValue() { X = 0, Y = this.EPV };
    //            case "Lat":
    //                return new DataValue() { X = 0, Y = this.Lat };
    //            case "Lon":
    //                return new DataValue() { X = 0, Y = this.Lon };
    //            case "Alt":
    //                return new DataValue() { X = 0, Y = this.Alt };
    //            case "VelN":
    //                return new DataValue() { X = 0, Y = this.VelN };
    //            case "VelE":
    //                return new DataValue() { X = 0, Y = this.VelE };
    //            case "VelD":
    //                return new DataValue() { X = 0, Y = this.VelD };
    //            case "Cog":
    //                return new DataValue() { X = 0, Y = this.Cog };
    //            case "nSat":
    //                return new DataValue() { X = 0, Y = this.nSat };
    //            case "SNR":
    //                return new DataValue() { X = 0, Y = this.SNR };
    //            case "N":
    //                return new DataValue() { X = 0, Y = this.N };
    //            case "J":
    //                return new DataValue() { X = 0, Y = this.J };
    //        }
    //        return null;
    //    }
    //    public UInt64 GPSTime;
    //    public byte Fix;
    //    public float EPH;
    //    public float EPV;
    //    public double Lat;
    //    public double Lon;
    //    public float Alt;
    //    public float VelN;
    //    public float VelE;
    //    public float VelD;
    //    public float Cog;
    //    public byte nSat;
    //    public UInt16 SNR;
    //    public UInt16 N;
    //    public UInt16 J;

    //    public static LogEntryDGPS Read(BinaryReader reader)
    //    {
    //        LogEntryDGPS result = new LogEntryDGPS();
    //        result.GPSTime = reader.ReadUInt64();
    //        result.Fix = reader.ReadByte();
    //        result.EPH = reader.ReadSingle();
    //        result.EPV = reader.ReadSingle();
    //        result.Lat = reader.ReadInt32() / 10000000.0;
    //        result.Lon = reader.ReadInt32() / 10000000.0;
    //        result.Alt = reader.ReadSingle();
    //        result.VelN = reader.ReadSingle();
    //        result.VelE = reader.ReadSingle();
    //        result.VelD = reader.ReadSingle();
    //        result.Cog = reader.ReadSingle();
    //        result.nSat = reader.ReadByte();
    //        result.SNR = reader.ReadUInt16();
    //        result.N = reader.ReadUInt16();
    //        result.J = reader.ReadUInt16();
    //        return result;
    //    }
    //}

    //// FMT ATTC, 19, ffff, Roll,Pitch,Yaw,Thrust
    //class LogEntryATTC : LogEntry
    //{
    //    public override string GetName() { return "ATTC"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Roll":
    //                return new DataValue() { X = 0, Y = this.Roll };
    //            case "Pitch":
    //                return new DataValue() { X = 0, Y = this.Pitch };
    //            case "Yaw":
    //                return new DataValue() { X = 0, Y = this.Yaw };
    //            case "Thrust":
    //                return new DataValue() { X = 0, Y = this.Thrust };
    //        }
    //        return null;
    //    }
    //    public float Roll;
    //    public float Pitch;
    //    public float Yaw;
    //    public float Thrust;

    //    public static LogEntryATTC Read(BinaryReader reader)
    //    {
    //        LogEntryATTC result = new LogEntryATTC();
    //        result.Roll = reader.ReadSingle();
    //        result.Pitch = reader.ReadSingle();
    //        result.Yaw = reader.ReadSingle();
    //        result.Thrust = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT ATC1, 19, ffff, Roll,Pitch,Yaw,Thrust
    //class LogEntryATC1 : LogEntry
    //{
    //    public override string GetName() { return "ATC1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Roll":
    //                return new DataValue() { X = 0, Y = this.Roll };
    //            case "Pitch":
    //                return new DataValue() { X = 0, Y = this.Pitch };
    //            case "Yaw":
    //                return new DataValue() { X = 0, Y = this.Yaw };
    //            case "Thrust":
    //                return new DataValue() { X = 0, Y = this.Thrust };
    //        }
    //        return null;
    //    }
    //    public float Roll;
    //    public float Pitch;
    //    public float Yaw;
    //    public float Thrust;

    //    public static LogEntryATC1 Read(BinaryReader reader)
    //    {
    //        LogEntryATC1 result = new LogEntryATC1();
    //        result.Roll = reader.ReadSingle();
    //        result.Pitch = reader.ReadSingle();
    //        result.Yaw = reader.ReadSingle();
    //        result.Thrust = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT STAT, 8, BBBBB, MainState,NavState,ArmS,Failsafe,IsRotWing
    //class LogEntrySTAT : LogEntry
    //{
    //    public override string GetName() { return "STAT"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "MainState":
    //                return new DataValue() { X = 0, Y = this.MainState };
    //            case "NavState":
    //                return new DataValue() { X = 0, Y = this.NavState };
    //            case "ArmS":
    //                return new DataValue() { X = 0, Y = this.ArmS };
    //            case "Failsafe":
    //                return new DataValue() { X = 0, Y = this.Failsafe };
    //            case "IsRotWing":
    //                return new DataValue() { X = 0, Y = this.IsRotWing };
    //        }
    //        return null;
    //    }
    //    public byte MainState;
    //    public byte NavState;
    //    public byte ArmS;
    //    public byte Failsafe;
    //    public byte IsRotWing;

    //    public static LogEntrySTAT Read(BinaryReader reader)
    //    {
    //        LogEntrySTAT result = new LogEntrySTAT();
    //        result.MainState = reader.ReadByte();
    //        result.NavState = reader.ReadByte();
    //        result.ArmS = reader.ReadByte();
    //        result.Failsafe = reader.ReadByte();
    //        result.IsRotWing = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT VTOL, 10, fBBB, Arsp,RwMode,TransMode,Failsafe
    //class LogEntryVTOL : LogEntry
    //{
    //    public override string GetName() { return "VTOL"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Arsp":
    //                return new DataValue() { X = 0, Y = this.Arsp };
    //            case "RwMode":
    //                return new DataValue() { X = 0, Y = this.RwMode };
    //            case "TransMode":
    //                return new DataValue() { X = 0, Y = this.TransMode };
    //            case "Failsafe":
    //                return new DataValue() { X = 0, Y = this.Failsafe };
    //        }
    //        return null;
    //    }
    //    public float Arsp;
    //    public byte RwMode;
    //    public byte TransMode;
    //    public byte Failsafe;

    //    public static LogEntryVTOL Read(BinaryReader reader)
    //    {
    //        LogEntryVTOL result = new LogEntryVTOL();
    //        result.Arsp = reader.ReadSingle();
    //        result.RwMode = reader.ReadByte();
    //        result.TransMode = reader.ReadByte();
    //        result.Failsafe = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT CTS, 31, fffffff, Vx_b,Vy_b,Vz_b,Vinf,P,Q,R
    //class LogEntryCTS : LogEntry
    //{
    //    public override string GetName() { return "CTS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Vx_b":
    //                return new DataValue() { X = 0, Y = this.Vx_b };
    //            case "Vy_b":
    //                return new DataValue() { X = 0, Y = this.Vy_b };
    //            case "Vz_b":
    //                return new DataValue() { X = 0, Y = this.Vz_b };
    //            case "Vinf":
    //                return new DataValue() { X = 0, Y = this.Vinf };
    //            case "P":
    //                return new DataValue() { X = 0, Y = this.P };
    //            case "Q":
    //                return new DataValue() { X = 0, Y = this.Q };
    //            case "R":
    //                return new DataValue() { X = 0, Y = this.R };
    //        }
    //        return null;
    //    }
    //    public float Vx_b;
    //    public float Vy_b;
    //    public float Vz_b;
    //    public float Vinf;
    //    public float P;
    //    public float Q;
    //    public float R;

    //    public static LogEntryCTS Read(BinaryReader reader)
    //    {
    //        LogEntryCTS result = new LogEntryCTS();
    //        result.Vx_b = reader.ReadSingle();
    //        result.Vy_b = reader.ReadSingle();
    //        result.Vz_b = reader.ReadSingle();
    //        result.Vinf = reader.ReadSingle();
    //        result.P = reader.ReadSingle();
    //        result.Q = reader.ReadSingle();
    //        result.R = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT RC, 58, ffffffffffffBBBL, C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,RSSI,CNT,Lost,Drop
    //class LogEntryRC : LogEntry
    //{
    //    public override string GetName() { return "RC"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "C0":
    //                return new DataValue() { X = 0, Y = this.C0 };
    //            case "C1":
    //                return new DataValue() { X = 0, Y = this.C1 };
    //            case "C2":
    //                return new DataValue() { X = 0, Y = this.C2 };
    //            case "C3":
    //                return new DataValue() { X = 0, Y = this.C3 };
    //            case "C4":
    //                return new DataValue() { X = 0, Y = this.C4 };
    //            case "C5":
    //                return new DataValue() { X = 0, Y = this.C5 };
    //            case "C6":
    //                return new DataValue() { X = 0, Y = this.C6 };
    //            case "C7":
    //                return new DataValue() { X = 0, Y = this.C7 };
    //            case "C8":
    //                return new DataValue() { X = 0, Y = this.C8 };
    //            case "C9":
    //                return new DataValue() { X = 0, Y = this.C9 };
    //            case "C10":
    //                return new DataValue() { X = 0, Y = this.C10 };
    //            case "C11":
    //                return new DataValue() { X = 0, Y = this.C11 };
    //            case "RSSI":
    //                return new DataValue() { X = 0, Y = this.RSSI };
    //            case "CNT":
    //                return new DataValue() { X = 0, Y = this.CNT };
    //            case "Lost":
    //                return new DataValue() { X = 0, Y = this.Lost };
    //            case "Drop":
    //                return new DataValue() { X = 0, Y = this.Drop };
    //        }
    //        return null;
    //    }
    //    public float C0;
    //    public float C1;
    //    public float C2;
    //    public float C3;
    //    public float C4;
    //    public float C5;
    //    public float C6;
    //    public float C7;
    //    public float C8;
    //    public float C9;
    //    public float C10;
    //    public float C11;
    //    public byte RSSI;
    //    public byte CNT;
    //    public byte Lost;
    //    public double Drop;

    //    public static LogEntryRC Read(BinaryReader reader)
    //    {
    //        LogEntryRC result = new LogEntryRC();
    //        result.C0 = reader.ReadSingle();
    //        result.C1 = reader.ReadSingle();
    //        result.C2 = reader.ReadSingle();
    //        result.C3 = reader.ReadSingle();
    //        result.C4 = reader.ReadSingle();
    //        result.C5 = reader.ReadSingle();
    //        result.C6 = reader.ReadSingle();
    //        result.C7 = reader.ReadSingle();
    //        result.C8 = reader.ReadSingle();
    //        result.C9 = reader.ReadSingle();
    //        result.C10 = reader.ReadSingle();
    //        result.C11 = reader.ReadSingle();
    //        result.RSSI = reader.ReadByte();
    //        result.CNT = reader.ReadByte();
    //        result.Lost = reader.ReadByte();
    //        result.Drop = reader.ReadInt32() / 10000000.0;
    //        return result;
    //    }
    //}

    //// FMT OUT0, 35, ffffffff, Out0,Out1,Out2,Out3,Out4,Out5,Out6,Out7
    //class LogEntryOUT0 : LogEntry
    //{
    //    public override string GetName() { return "OUT0"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Out0":
    //                return new DataValue() { X = 0, Y = this.Out0 };
    //            case "Out1":
    //                return new DataValue() { X = 0, Y = this.Out1 };
    //            case "Out2":
    //                return new DataValue() { X = 0, Y = this.Out2 };
    //            case "Out3":
    //                return new DataValue() { X = 0, Y = this.Out3 };
    //            case "Out4":
    //                return new DataValue() { X = 0, Y = this.Out4 };
    //            case "Out5":
    //                return new DataValue() { X = 0, Y = this.Out5 };
    //            case "Out6":
    //                return new DataValue() { X = 0, Y = this.Out6 };
    //            case "Out7":
    //                return new DataValue() { X = 0, Y = this.Out7 };
    //        }
    //        return null;
    //    }
    //    public float Out0;
    //    public float Out1;
    //    public float Out2;
    //    public float Out3;
    //    public float Out4;
    //    public float Out5;
    //    public float Out6;
    //    public float Out7;

    //    public static LogEntryOUT0 Read(BinaryReader reader)
    //    {
    //        LogEntryOUT0 result = new LogEntryOUT0();
    //        result.Out0 = reader.ReadSingle();
    //        result.Out1 = reader.ReadSingle();
    //        result.Out2 = reader.ReadSingle();
    //        result.Out3 = reader.ReadSingle();
    //        result.Out4 = reader.ReadSingle();
    //        result.Out5 = reader.ReadSingle();
    //        result.Out6 = reader.ReadSingle();
    //        result.Out7 = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT OUT1, 35, ffffffff, Out0,Out1,Out2,Out3,Out4,Out5,Out6,Out7
    //class LogEntryOUT1 : LogEntry
    //{
    //    public override string GetName() { return "OUT1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Out0":
    //                return new DataValue() { X = 0, Y = this.Out0 };
    //            case "Out1":
    //                return new DataValue() { X = 0, Y = this.Out1 };
    //            case "Out2":
    //                return new DataValue() { X = 0, Y = this.Out2 };
    //            case "Out3":
    //                return new DataValue() { X = 0, Y = this.Out3 };
    //            case "Out4":
    //                return new DataValue() { X = 0, Y = this.Out4 };
    //            case "Out5":
    //                return new DataValue() { X = 0, Y = this.Out5 };
    //            case "Out6":
    //                return new DataValue() { X = 0, Y = this.Out6 };
    //            case "Out7":
    //                return new DataValue() { X = 0, Y = this.Out7 };
    //        }
    //        return null;
    //    }
    //    public float Out0;
    //    public float Out1;
    //    public float Out2;
    //    public float Out3;
    //    public float Out4;
    //    public float Out5;
    //    public float Out6;
    //    public float Out7;

    //    public static LogEntryOUT1 Read(BinaryReader reader)
    //    {
    //        LogEntryOUT1 result = new LogEntryOUT1();
    //        result.Out0 = reader.ReadSingle();
    //        result.Out1 = reader.ReadSingle();
    //        result.Out2 = reader.ReadSingle();
    //        result.Out3 = reader.ReadSingle();
    //        result.Out4 = reader.ReadSingle();
    //        result.Out5 = reader.ReadSingle();
    //        result.Out6 = reader.ReadSingle();
    //        result.Out7 = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT AIRS, 15, fff, IndSpeed,TrueSpeed,AirTemp
    //class LogEntryAIRS : LogEntry
    //{
    //    public override string GetName() { return "AIRS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "IndSpeed":
    //                return new DataValue() { X = 0, Y = this.IndSpeed };
    //            case "TrueSpeed":
    //                return new DataValue() { X = 0, Y = this.TrueSpeed };
    //            case "AirTemp":
    //                return new DataValue() { X = 0, Y = this.AirTemp };
    //        }
    //        return null;
    //    }
    //    public float IndSpeed;
    //    public float TrueSpeed;
    //    public float AirTemp;

    //    public static LogEntryAIRS Read(BinaryReader reader)
    //    {
    //        LogEntryAIRS result = new LogEntryAIRS();
    //        result.IndSpeed = reader.ReadSingle();
    //        result.TrueSpeed = reader.ReadSingle();
    //        result.AirTemp = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT ARSP, 15, fff, RollRateSP,PitchRateSP,YawRateSP
    //class LogEntryARSP : LogEntry
    //{
    //    public override string GetName() { return "ARSP"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RollRateSP":
    //                return new DataValue() { X = 0, Y = this.RollRateSP };
    //            case "PitchRateSP":
    //                return new DataValue() { X = 0, Y = this.PitchRateSP };
    //            case "YawRateSP":
    //                return new DataValue() { X = 0, Y = this.YawRateSP };
    //        }
    //        return null;
    //    }
    //    public float RollRateSP;
    //    public float PitchRateSP;
    //    public float YawRateSP;

    //    public static LogEntryARSP Read(BinaryReader reader)
    //    {
    //        LogEntryARSP result = new LogEntryARSP();
    //        result.RollRateSP = reader.ReadSingle();
    //        result.PitchRateSP = reader.ReadSingle();
    //        result.YawRateSP = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT FLOW, 41, BffffffLLHhB, ID,RawX,RawY,RX,RY,RZ,Dist,TSpan,DtSonar,FrmCnt,GT,Qlty
    //class LogEntryFLOW : LogEntry
    //{
    //    public override string GetName() { return "FLOW"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "ID":
    //                return new DataValue() { X = 0, Y = this.ID };
    //            case "RawX":
    //                return new DataValue() { X = 0, Y = this.RawX };
    //            case "RawY":
    //                return new DataValue() { X = 0, Y = this.RawY };
    //            case "RX":
    //                return new DataValue() { X = 0, Y = this.RX };
    //            case "RY":
    //                return new DataValue() { X = 0, Y = this.RY };
    //            case "RZ":
    //                return new DataValue() { X = 0, Y = this.RZ };
    //            case "Dist":
    //                return new DataValue() { X = 0, Y = this.Dist };
    //            case "TSpan":
    //                return new DataValue() { X = 0, Y = this.TSpan };
    //            case "DtSonar":
    //                return new DataValue() { X = 0, Y = this.DtSonar };
    //            case "FrmCnt":
    //                return new DataValue() { X = 0, Y = this.FrmCnt };
    //            case "GT":
    //                return new DataValue() { X = 0, Y = this.GT };
    //            case "Qlty":
    //                return new DataValue() { X = 0, Y = this.Qlty };
    //        }
    //        return null;
    //    }
    //    public byte ID;
    //    public float RawX;
    //    public float RawY;
    //    public float RX;
    //    public float RY;
    //    public float RZ;
    //    public float Dist;
    //    public double TSpan;
    //    public double DtSonar;
    //    public UInt16 FrmCnt;
    //    public Int16 GT;
    //    public byte Qlty;

    //    public static LogEntryFLOW Read(BinaryReader reader)
    //    {
    //        LogEntryFLOW result = new LogEntryFLOW();
    //        result.ID = reader.ReadByte();
    //        result.RawX = reader.ReadSingle();
    //        result.RawY = reader.ReadSingle();
    //        result.RX = reader.ReadSingle();
    //        result.RY = reader.ReadSingle();
    //        result.RZ = reader.ReadSingle();
    //        result.Dist = reader.ReadSingle();
    //        result.TSpan = reader.ReadInt32() / 10000000.0;
    //        result.DtSonar = reader.ReadInt32() / 10000000.0;
    //        result.FrmCnt = reader.ReadUInt16();
    //        result.GT = reader.ReadInt16();
    //        result.Qlty = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT GPOS, 39, LLfffffff, Lat,Lon,Alt,VelN,VelE,VelD,EPH,EPV,TALT
    //class LogEntryGPOS : LogEntry
    //{
    //    public override string GetName() { return "GPOS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Lat":
    //                return new DataValue() { X = 0, Y = this.Lat };
    //            case "Lon":
    //                return new DataValue() { X = 0, Y = this.Lon };
    //            case "Alt":
    //                return new DataValue() { X = 0, Y = this.Alt };
    //            case "VelN":
    //                return new DataValue() { X = 0, Y = this.VelN };
    //            case "VelE":
    //                return new DataValue() { X = 0, Y = this.VelE };
    //            case "VelD":
    //                return new DataValue() { X = 0, Y = this.VelD };
    //            case "EPH":
    //                return new DataValue() { X = 0, Y = this.EPH };
    //            case "EPV":
    //                return new DataValue() { X = 0, Y = this.EPV };
    //            case "TALT":
    //                return new DataValue() { X = 0, Y = this.TALT };
    //        }
    //        return null;
    //    }
    //    public double Lat;
    //    public double Lon;
    //    public float Alt;
    //    public float VelN;
    //    public float VelE;
    //    public float VelD;
    //    public float EPH;
    //    public float EPV;
    //    public float TALT;

    //    public static LogEntryGPOS Read(BinaryReader reader)
    //    {
    //        LogEntryGPOS result = new LogEntryGPOS();
    //        result.Lat = reader.ReadInt32() / 10000000.0;
    //        result.Lon = reader.ReadInt32() / 10000000.0;
    //        result.Alt = reader.ReadSingle();
    //        result.VelN = reader.ReadSingle();
    //        result.VelE = reader.ReadSingle();
    //        result.VelD = reader.ReadSingle();
    //        result.EPH = reader.ReadSingle();
    //        result.EPV = reader.ReadSingle();
    //        result.TALT = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT GPSP, 30, BLLffBfbf, NavState,Lat,Lon,Alt,Yaw,Type,LoitR,LoitDir,PitMin
    //class LogEntryGPSP : LogEntry
    //{
    //    public override string GetName() { return "GPSP"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "NavState":
    //                return new DataValue() { X = 0, Y = this.NavState };
    //            case "Lat":
    //                return new DataValue() { X = 0, Y = this.Lat };
    //            case "Lon":
    //                return new DataValue() { X = 0, Y = this.Lon };
    //            case "Alt":
    //                return new DataValue() { X = 0, Y = this.Alt };
    //            case "Yaw":
    //                return new DataValue() { X = 0, Y = this.Yaw };
    //            case "Type":
    //                return new DataValue() { X = 0, Y = this.Type };
    //            case "LoitR":
    //                return new DataValue() { X = 0, Y = this.LoitR };
    //            case "LoitDir":
    //                return new DataValue() { X = 0, Y = this.LoitDir };
    //            case "PitMin":
    //                return new DataValue() { X = 0, Y = this.PitMin };
    //        }
    //        return null;
    //    }
    //    public byte NavState;
    //    public double Lat;
    //    public double Lon;
    //    public float Alt;
    //    public float Yaw;
    //    public byte Type;
    //    public float LoitR;
    //    public sbyte LoitDir;
    //    public float PitMin;

    //    public static LogEntryGPSP Read(BinaryReader reader)
    //    {
    //        LogEntryGPSP result = new LogEntryGPSP();
    //        result.NavState = reader.ReadByte();
    //        result.Lat = reader.ReadInt32() / 10000000.0;
    //        result.Lon = reader.ReadInt32() / 10000000.0;
    //        result.Alt = reader.ReadSingle();
    //        result.Yaw = reader.ReadSingle();
    //        result.Type = reader.ReadByte();
    //        result.LoitR = reader.ReadSingle();
    //        result.LoitDir = (sbyte)reader.ReadByte();
    //        result.PitMin = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT ESC, 34, HBBBHHffiffH, count,nESC,Conn,N,Ver,Adr,Volt,Amp,RPM,Temp,SetP,SetPRAW
    //class LogEntryESC : LogEntry
    //{
    //    public override string GetName() { return "ESC"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "count":
    //                return new DataValue() { X = 0, Y = this.count };
    //            case "nESC":
    //                return new DataValue() { X = 0, Y = this.nESC };
    //            case "Conn":
    //                return new DataValue() { X = 0, Y = this.Conn };
    //            case "N":
    //                return new DataValue() { X = 0, Y = this.N };
    //            case "Ver":
    //                return new DataValue() { X = 0, Y = this.Ver };
    //            case "Adr":
    //                return new DataValue() { X = 0, Y = this.Adr };
    //            case "Volt":
    //                return new DataValue() { X = 0, Y = this.Volt };
    //            case "Amp":
    //                return new DataValue() { X = 0, Y = this.Amp };
    //            case "RPM":
    //                return new DataValue() { X = 0, Y = this.RPM };
    //            case "Temp":
    //                return new DataValue() { X = 0, Y = this.Temp };
    //            case "SetP":
    //                return new DataValue() { X = 0, Y = this.SetP };
    //            case "SetPRAW":
    //                return new DataValue() { X = 0, Y = this.SetPRAW };
    //        }
    //        return null;
    //    }
    //    public UInt16 count;
    //    public byte nESC;
    //    public byte Conn;
    //    public byte N;
    //    public UInt16 Ver;
    //    public UInt16 Adr;
    //    public float Volt;
    //    public float Amp;
    //    public Int32 RPM;
    //    public float Temp;
    //    public float SetP;
    //    public UInt16 SetPRAW;

    //    public static LogEntryESC Read(BinaryReader reader)
    //    {
    //        LogEntryESC result = new LogEntryESC();
    //        result.count = reader.ReadUInt16();
    //        result.nESC = reader.ReadByte();
    //        result.Conn = reader.ReadByte();
    //        result.N = reader.ReadByte();
    //        result.Ver = reader.ReadUInt16();
    //        result.Adr = reader.ReadUInt16();
    //        result.Volt = reader.ReadSingle();
    //        result.Amp = reader.ReadSingle();
    //        result.RPM = reader.ReadInt32();
    //        result.Temp = reader.ReadSingle();
    //        result.SetP = reader.ReadSingle();
    //        result.SetPRAW = reader.ReadUInt16();
    //        return result;
    //    }
    //}

    //// FMT GVSP, 15, fff, VX,VY,VZ
    //class LogEntryGVSP : LogEntry
    //{
    //    public override string GetName() { return "GVSP"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "VX":
    //                return new DataValue() { X = 0, Y = this.VX };
    //            case "VY":
    //                return new DataValue() { X = 0, Y = this.VY };
    //            case "VZ":
    //                return new DataValue() { X = 0, Y = this.VZ };
    //        }
    //        return null;
    //    }
    //    public float VX;
    //    public float VY;
    //    public float VZ;

    //    public static LogEntryGVSP Read(BinaryReader reader)
    //    {
    //        LogEntryGVSP result = new LogEntryGVSP();
    //        result.VX = reader.ReadSingle();
    //        result.VY = reader.ReadSingle();
    //        result.VZ = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT BATT, 28, ffffffB, V,VFilt,C,CFilt,Discharged,Remaining,Warning
    //class LogEntryBATT : LogEntry
    //{
    //    public override string GetName() { return "BATT"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "V":
    //                return new DataValue() { X = 0, Y = this.V };
    //            case "VFilt":
    //                return new DataValue() { X = 0, Y = this.VFilt };
    //            case "C":
    //                return new DataValue() { X = 0, Y = this.C };
    //            case "CFilt":
    //                return new DataValue() { X = 0, Y = this.CFilt };
    //            case "Discharged":
    //                return new DataValue() { X = 0, Y = this.Discharged };
    //            case "Remaining":
    //                return new DataValue() { X = 0, Y = this.Remaining };
    //            case "Warning":
    //                return new DataValue() { X = 0, Y = this.Warning };
    //        }
    //        return null;
    //    }
    //    public float V;
    //    public float VFilt;
    //    public float C;
    //    public float CFilt;
    //    public float Discharged;
    //    public float Remaining;
    //    public byte Warning;

    //    public static LogEntryBATT Read(BinaryReader reader)
    //    {
    //        LogEntryBATT result = new LogEntryBATT();
    //        result.V = reader.ReadSingle();
    //        result.VFilt = reader.ReadSingle();
    //        result.C = reader.ReadSingle();
    //        result.CFilt = reader.ReadSingle();
    //        result.Discharged = reader.ReadSingle();
    //        result.Remaining = reader.ReadSingle();
    //        result.Warning = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT DIST, 14, BBBff, Id,Type,Orientation,Distance,Covariance
    //class LogEntryDIST : LogEntry
    //{
    //    public override string GetName() { return "DIST"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Id":
    //                return new DataValue() { X = 0, Y = this.Id };
    //            case "Type":
    //                return new DataValue() { X = 0, Y = this.Type };
    //            case "Orientation":
    //                return new DataValue() { X = 0, Y = this.Orientation };
    //            case "Distance":
    //                return new DataValue() { X = 0, Y = this.Distance };
    //            case "Covariance":
    //                return new DataValue() { X = 0, Y = this.Covariance };
    //        }
    //        return null;
    //    }
    //    public byte Id;
    //    public byte Type;
    //    public byte Orientation;
    //    public float Distance;
    //    public float Covariance;

    //    public static LogEntryDIST Read(BinaryReader reader)
    //    {
    //        LogEntryDIST result = new LogEntryDIST();
    //        result.Id = reader.ReadByte();
    //        result.Type = reader.ReadByte();
    //        result.Orientation = reader.ReadByte();
    //        result.Distance = reader.ReadSingle();
    //        result.Covariance = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT TEL0, 20, BBBBHHBQ, RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime
    //class LogEntryTEL0 : LogEntry
    //{
    //    public override string GetName() { return "TEL0"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RSSI":
    //                return new DataValue() { X = 0, Y = this.RSSI };
    //            case "RemRSSI":
    //                return new DataValue() { X = 0, Y = this.RemRSSI };
    //            case "Noise":
    //                return new DataValue() { X = 0, Y = this.Noise };
    //            case "RemNoise":
    //                return new DataValue() { X = 0, Y = this.RemNoise };
    //            case "RXErr":
    //                return new DataValue() { X = 0, Y = this.RXErr };
    //            case "Fixed":
    //                return new DataValue() { X = 0, Y = this.Fixed };
    //            case "TXBuf":
    //                return new DataValue() { X = 0, Y = this.TXBuf };
    //            case "HbTime":
    //                return new DataValue() { X = 0, Y = this.HbTime };
    //        }
    //        return null;
    //    }
    //    public byte RSSI;
    //    public byte RemRSSI;
    //    public byte Noise;
    //    public byte RemNoise;
    //    public UInt16 RXErr;
    //    public UInt16 Fixed;
    //    public byte TXBuf;
    //    public UInt64 HbTime;

    //    public static LogEntryTEL0 Read(BinaryReader reader)
    //    {
    //        LogEntryTEL0 result = new LogEntryTEL0();
    //        result.RSSI = reader.ReadByte();
    //        result.RemRSSI = reader.ReadByte();
    //        result.Noise = reader.ReadByte();
    //        result.RemNoise = reader.ReadByte();
    //        result.RXErr = reader.ReadUInt16();
    //        result.Fixed = reader.ReadUInt16();
    //        result.TXBuf = reader.ReadByte();
    //        result.HbTime = reader.ReadUInt64();
    //        return result;
    //    }
    //}

    //// FMT TEL1, 20, BBBBHHBQ, RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime
    //class LogEntryTEL1 : LogEntry
    //{
    //    public override string GetName() { return "TEL1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RSSI":
    //                return new DataValue() { X = 0, Y = this.RSSI };
    //            case "RemRSSI":
    //                return new DataValue() { X = 0, Y = this.RemRSSI };
    //            case "Noise":
    //                return new DataValue() { X = 0, Y = this.Noise };
    //            case "RemNoise":
    //                return new DataValue() { X = 0, Y = this.RemNoise };
    //            case "RXErr":
    //                return new DataValue() { X = 0, Y = this.RXErr };
    //            case "Fixed":
    //                return new DataValue() { X = 0, Y = this.Fixed };
    //            case "TXBuf":
    //                return new DataValue() { X = 0, Y = this.TXBuf };
    //            case "HbTime":
    //                return new DataValue() { X = 0, Y = this.HbTime };
    //        }
    //        return null;
    //    }
    //    public byte RSSI;
    //    public byte RemRSSI;
    //    public byte Noise;
    //    public byte RemNoise;
    //    public UInt16 RXErr;
    //    public UInt16 Fixed;
    //    public byte TXBuf;
    //    public UInt64 HbTime;

    //    public static LogEntryTEL1 Read(BinaryReader reader)
    //    {
    //        LogEntryTEL1 result = new LogEntryTEL1();
    //        result.RSSI = reader.ReadByte();
    //        result.RemRSSI = reader.ReadByte();
    //        result.Noise = reader.ReadByte();
    //        result.RemNoise = reader.ReadByte();
    //        result.RXErr = reader.ReadUInt16();
    //        result.Fixed = reader.ReadUInt16();
    //        result.TXBuf = reader.ReadByte();
    //        result.HbTime = reader.ReadUInt64();
    //        return result;
    //    }
    //}

    //// FMT TEL2, 20, BBBBHHBQ, RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime
    //class LogEntryTEL2 : LogEntry
    //{
    //    public override string GetName() { return "TEL2"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RSSI":
    //                return new DataValue() { X = 0, Y = this.RSSI };
    //            case "RemRSSI":
    //                return new DataValue() { X = 0, Y = this.RemRSSI };
    //            case "Noise":
    //                return new DataValue() { X = 0, Y = this.Noise };
    //            case "RemNoise":
    //                return new DataValue() { X = 0, Y = this.RemNoise };
    //            case "RXErr":
    //                return new DataValue() { X = 0, Y = this.RXErr };
    //            case "Fixed":
    //                return new DataValue() { X = 0, Y = this.Fixed };
    //            case "TXBuf":
    //                return new DataValue() { X = 0, Y = this.TXBuf };
    //            case "HbTime":
    //                return new DataValue() { X = 0, Y = this.HbTime };
    //        }
    //        return null;
    //    }
    //    public byte RSSI;
    //    public byte RemRSSI;
    //    public byte Noise;
    //    public byte RemNoise;
    //    public UInt16 RXErr;
    //    public UInt16 Fixed;
    //    public byte TXBuf;
    //    public UInt64 HbTime;

    //    public static LogEntryTEL2 Read(BinaryReader reader)
    //    {
    //        LogEntryTEL2 result = new LogEntryTEL2();
    //        result.RSSI = reader.ReadByte();
    //        result.RemRSSI = reader.ReadByte();
    //        result.Noise = reader.ReadByte();
    //        result.RemNoise = reader.ReadByte();
    //        result.RXErr = reader.ReadUInt16();
    //        result.Fixed = reader.ReadUInt16();
    //        result.TXBuf = reader.ReadByte();
    //        result.HbTime = reader.ReadUInt64();
    //        return result;
    //    }
    //}

    //// FMT TEL3, 20, BBBBHHBQ, RSSI,RemRSSI,Noise,RemNoise,RXErr,Fixed,TXBuf,HbTime
    //class LogEntryTEL3 : LogEntry
    //{
    //    public override string GetName() { return "TEL3"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RSSI":
    //                return new DataValue() { X = 0, Y = this.RSSI };
    //            case "RemRSSI":
    //                return new DataValue() { X = 0, Y = this.RemRSSI };
    //            case "Noise":
    //                return new DataValue() { X = 0, Y = this.Noise };
    //            case "RemNoise":
    //                return new DataValue() { X = 0, Y = this.RemNoise };
    //            case "RXErr":
    //                return new DataValue() { X = 0, Y = this.RXErr };
    //            case "Fixed":
    //                return new DataValue() { X = 0, Y = this.Fixed };
    //            case "TXBuf":
    //                return new DataValue() { X = 0, Y = this.TXBuf };
    //            case "HbTime":
    //                return new DataValue() { X = 0, Y = this.HbTime };
    //        }
    //        return null;
    //    }
    //    public byte RSSI;
    //    public byte RemRSSI;
    //    public byte Noise;
    //    public byte RemNoise;
    //    public UInt16 RXErr;
    //    public UInt16 Fixed;
    //    public byte TXBuf;
    //    public UInt64 HbTime;

    //    public static LogEntryTEL3 Read(BinaryReader reader)
    //    {
    //        LogEntryTEL3 result = new LogEntryTEL3();
    //        result.RSSI = reader.ReadByte();
    //        result.RemRSSI = reader.ReadByte();
    //        result.Noise = reader.ReadByte();
    //        result.RemNoise = reader.ReadByte();
    //        result.RXErr = reader.ReadUInt16();
    //        result.Fixed = reader.ReadUInt16();
    //        result.TXBuf = reader.ReadByte();
    //        result.HbTime = reader.ReadUInt64();
    //        return result;
    //    }
    //}

    //// FMT EST0, 56, ffffffffffffBBHB, s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,nStat,fNaN,fFault,fTOut
    //class LogEntryEST0 : LogEntry
    //{
    //    public override string GetName() { return "EST0"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "s0":
    //                return new DataValue() { X = 0, Y = this.s0 };
    //            case "s1":
    //                return new DataValue() { X = 0, Y = this.s1 };
    //            case "s2":
    //                return new DataValue() { X = 0, Y = this.s2 };
    //            case "s3":
    //                return new DataValue() { X = 0, Y = this.s3 };
    //            case "s4":
    //                return new DataValue() { X = 0, Y = this.s4 };
    //            case "s5":
    //                return new DataValue() { X = 0, Y = this.s5 };
    //            case "s6":
    //                return new DataValue() { X = 0, Y = this.s6 };
    //            case "s7":
    //                return new DataValue() { X = 0, Y = this.s7 };
    //            case "s8":
    //                return new DataValue() { X = 0, Y = this.s8 };
    //            case "s9":
    //                return new DataValue() { X = 0, Y = this.s9 };
    //            case "s10":
    //                return new DataValue() { X = 0, Y = this.s10 };
    //            case "s11":
    //                return new DataValue() { X = 0, Y = this.s11 };
    //            case "nStat":
    //                return new DataValue() { X = 0, Y = this.nStat };
    //            case "fNaN":
    //                return new DataValue() { X = 0, Y = this.fNaN };
    //            case "fFault":
    //                return new DataValue() { X = 0, Y = this.fFault };
    //            case "fTOut":
    //                return new DataValue() { X = 0, Y = this.fTOut };
    //        }
    //        return null;
    //    }
    //    public float s0;
    //    public float s1;
    //    public float s2;
    //    public float s3;
    //    public float s4;
    //    public float s5;
    //    public float s6;
    //    public float s7;
    //    public float s8;
    //    public float s9;
    //    public float s10;
    //    public float s11;
    //    public byte nStat;
    //    public byte fNaN;
    //    public UInt16 fFault;
    //    public byte fTOut;

    //    public static LogEntryEST0 Read(BinaryReader reader)
    //    {
    //        LogEntryEST0 result = new LogEntryEST0();
    //        result.s0 = reader.ReadSingle();
    //        result.s1 = reader.ReadSingle();
    //        result.s2 = reader.ReadSingle();
    //        result.s3 = reader.ReadSingle();
    //        result.s4 = reader.ReadSingle();
    //        result.s5 = reader.ReadSingle();
    //        result.s6 = reader.ReadSingle();
    //        result.s7 = reader.ReadSingle();
    //        result.s8 = reader.ReadSingle();
    //        result.s9 = reader.ReadSingle();
    //        result.s10 = reader.ReadSingle();
    //        result.s11 = reader.ReadSingle();
    //        result.nStat = reader.ReadByte();
    //        result.fNaN = reader.ReadByte();
    //        result.fFault = reader.ReadUInt16();
    //        result.fTOut = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT EST1, 67, ffffffffffffffff, s12,s13,s14,s15,s16,s17,s18,s19,s20,s21,s22,s23,s24,s25,s26,s27
    //class LogEntryEST1 : LogEntry
    //{
    //    public override string GetName() { return "EST1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "s12":
    //                return new DataValue() { X = 0, Y = this.s12 };
    //            case "s13":
    //                return new DataValue() { X = 0, Y = this.s13 };
    //            case "s14":
    //                return new DataValue() { X = 0, Y = this.s14 };
    //            case "s15":
    //                return new DataValue() { X = 0, Y = this.s15 };
    //            case "s16":
    //                return new DataValue() { X = 0, Y = this.s16 };
    //            case "s17":
    //                return new DataValue() { X = 0, Y = this.s17 };
    //            case "s18":
    //                return new DataValue() { X = 0, Y = this.s18 };
    //            case "s19":
    //                return new DataValue() { X = 0, Y = this.s19 };
    //            case "s20":
    //                return new DataValue() { X = 0, Y = this.s20 };
    //            case "s21":
    //                return new DataValue() { X = 0, Y = this.s21 };
    //            case "s22":
    //                return new DataValue() { X = 0, Y = this.s22 };
    //            case "s23":
    //                return new DataValue() { X = 0, Y = this.s23 };
    //            case "s24":
    //                return new DataValue() { X = 0, Y = this.s24 };
    //            case "s25":
    //                return new DataValue() { X = 0, Y = this.s25 };
    //            case "s26":
    //                return new DataValue() { X = 0, Y = this.s26 };
    //            case "s27":
    //                return new DataValue() { X = 0, Y = this.s27 };
    //        }
    //        return null;
    //    }
    //    public float s12;
    //    public float s13;
    //    public float s14;
    //    public float s15;
    //    public float s16;
    //    public float s17;
    //    public float s18;
    //    public float s19;
    //    public float s20;
    //    public float s21;
    //    public float s22;
    //    public float s23;
    //    public float s24;
    //    public float s25;
    //    public float s26;
    //    public float s27;

    //    public static LogEntryEST1 Read(BinaryReader reader)
    //    {
    //        LogEntryEST1 result = new LogEntryEST1();
    //        result.s12 = reader.ReadSingle();
    //        result.s13 = reader.ReadSingle();
    //        result.s14 = reader.ReadSingle();
    //        result.s15 = reader.ReadSingle();
    //        result.s16 = reader.ReadSingle();
    //        result.s17 = reader.ReadSingle();
    //        result.s18 = reader.ReadSingle();
    //        result.s19 = reader.ReadSingle();
    //        result.s20 = reader.ReadSingle();
    //        result.s21 = reader.ReadSingle();
    //        result.s22 = reader.ReadSingle();
    //        result.s23 = reader.ReadSingle();
    //        result.s24 = reader.ReadSingle();
    //        result.s25 = reader.ReadSingle();
    //        result.s26 = reader.ReadSingle();
    //        result.s27 = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT EST2, 56, ffffffffffffHHB, P0,P1,P2,P3,P4,P5,P6,P7,P8,P9,P10,P11,GCHK,CTRL,fHealth
    //class LogEntryEST2 : LogEntry
    //{
    //    public override string GetName() { return "EST2"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "P0":
    //                return new DataValue() { X = 0, Y = this.P0 };
    //            case "P1":
    //                return new DataValue() { X = 0, Y = this.P1 };
    //            case "P2":
    //                return new DataValue() { X = 0, Y = this.P2 };
    //            case "P3":
    //                return new DataValue() { X = 0, Y = this.P3 };
    //            case "P4":
    //                return new DataValue() { X = 0, Y = this.P4 };
    //            case "P5":
    //                return new DataValue() { X = 0, Y = this.P5 };
    //            case "P6":
    //                return new DataValue() { X = 0, Y = this.P6 };
    //            case "P7":
    //                return new DataValue() { X = 0, Y = this.P7 };
    //            case "P8":
    //                return new DataValue() { X = 0, Y = this.P8 };
    //            case "P9":
    //                return new DataValue() { X = 0, Y = this.P9 };
    //            case "P10":
    //                return new DataValue() { X = 0, Y = this.P10 };
    //            case "P11":
    //                return new DataValue() { X = 0, Y = this.P11 };
    //            case "GCHK":
    //                return new DataValue() { X = 0, Y = this.GCHK };
    //            case "CTRL":
    //                return new DataValue() { X = 0, Y = this.CTRL };
    //            case "fHealth":
    //                return new DataValue() { X = 0, Y = this.fHealth };
    //        }
    //        return null;
    //    }
    //    public float P0;
    //    public float P1;
    //    public float P2;
    //    public float P3;
    //    public float P4;
    //    public float P5;
    //    public float P6;
    //    public float P7;
    //    public float P8;
    //    public float P9;
    //    public float P10;
    //    public float P11;
    //    public UInt16 GCHK;
    //    public UInt16 CTRL;
    //    public byte fHealth;

    //    public static LogEntryEST2 Read(BinaryReader reader)
    //    {
    //        LogEntryEST2 result = new LogEntryEST2();
    //        result.P0 = reader.ReadSingle();
    //        result.P1 = reader.ReadSingle();
    //        result.P2 = reader.ReadSingle();
    //        result.P3 = reader.ReadSingle();
    //        result.P4 = reader.ReadSingle();
    //        result.P5 = reader.ReadSingle();
    //        result.P6 = reader.ReadSingle();
    //        result.P7 = reader.ReadSingle();
    //        result.P8 = reader.ReadSingle();
    //        result.P9 = reader.ReadSingle();
    //        result.P10 = reader.ReadSingle();
    //        result.P11 = reader.ReadSingle();
    //        result.GCHK = reader.ReadUInt16();
    //        result.CTRL = reader.ReadUInt16();
    //        result.fHealth = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT EST3, 67, ffffffffffffffff, P12,P13,P14,P15,P16,P17,P18,P19,P20,P21,P22,P23,P24,P25,P26,P27
    //class LogEntryEST3 : LogEntry
    //{
    //    public override string GetName() { return "EST3"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "P12":
    //                return new DataValue() { X = 0, Y = this.P12 };
    //            case "P13":
    //                return new DataValue() { X = 0, Y = this.P13 };
    //            case "P14":
    //                return new DataValue() { X = 0, Y = this.P14 };
    //            case "P15":
    //                return new DataValue() { X = 0, Y = this.P15 };
    //            case "P16":
    //                return new DataValue() { X = 0, Y = this.P16 };
    //            case "P17":
    //                return new DataValue() { X = 0, Y = this.P17 };
    //            case "P18":
    //                return new DataValue() { X = 0, Y = this.P18 };
    //            case "P19":
    //                return new DataValue() { X = 0, Y = this.P19 };
    //            case "P20":
    //                return new DataValue() { X = 0, Y = this.P20 };
    //            case "P21":
    //                return new DataValue() { X = 0, Y = this.P21 };
    //            case "P22":
    //                return new DataValue() { X = 0, Y = this.P22 };
    //            case "P23":
    //                return new DataValue() { X = 0, Y = this.P23 };
    //            case "P24":
    //                return new DataValue() { X = 0, Y = this.P24 };
    //            case "P25":
    //                return new DataValue() { X = 0, Y = this.P25 };
    //            case "P26":
    //                return new DataValue() { X = 0, Y = this.P26 };
    //            case "P27":
    //                return new DataValue() { X = 0, Y = this.P27 };
    //        }
    //        return null;
    //    }
    //    public float P12;
    //    public float P13;
    //    public float P14;
    //    public float P15;
    //    public float P16;
    //    public float P17;
    //    public float P18;
    //    public float P19;
    //    public float P20;
    //    public float P21;
    //    public float P22;
    //    public float P23;
    //    public float P24;
    //    public float P25;
    //    public float P26;
    //    public float P27;

    //    public static LogEntryEST3 Read(BinaryReader reader)
    //    {
    //        LogEntryEST3 result = new LogEntryEST3();
    //        result.P12 = reader.ReadSingle();
    //        result.P13 = reader.ReadSingle();
    //        result.P14 = reader.ReadSingle();
    //        result.P15 = reader.ReadSingle();
    //        result.P16 = reader.ReadSingle();
    //        result.P17 = reader.ReadSingle();
    //        result.P18 = reader.ReadSingle();
    //        result.P19 = reader.ReadSingle();
    //        result.P20 = reader.ReadSingle();
    //        result.P21 = reader.ReadSingle();
    //        result.P22 = reader.ReadSingle();
    //        result.P23 = reader.ReadSingle();
    //        result.P24 = reader.ReadSingle();
    //        result.P25 = reader.ReadSingle();
    //        result.P26 = reader.ReadSingle();
    //        result.P27 = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT EST4, 51, ffffffffffff, VxI,VyI,VzI,PxI,PyI,PzI,VxIV,VyIV,VzIV,PxIV,PyIV,PzIV
    //class LogEntryEST4 : LogEntry
    //{
    //    public override string GetName() { return "EST4"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "VxI":
    //                return new DataValue() { X = 0, Y = this.VxI };
    //            case "VyI":
    //                return new DataValue() { X = 0, Y = this.VyI };
    //            case "VzI":
    //                return new DataValue() { X = 0, Y = this.VzI };
    //            case "PxI":
    //                return new DataValue() { X = 0, Y = this.PxI };
    //            case "PyI":
    //                return new DataValue() { X = 0, Y = this.PyI };
    //            case "PzI":
    //                return new DataValue() { X = 0, Y = this.PzI };
    //            case "VxIV":
    //                return new DataValue() { X = 0, Y = this.VxIV };
    //            case "VyIV":
    //                return new DataValue() { X = 0, Y = this.VyIV };
    //            case "VzIV":
    //                return new DataValue() { X = 0, Y = this.VzIV };
    //            case "PxIV":
    //                return new DataValue() { X = 0, Y = this.PxIV };
    //            case "PyIV":
    //                return new DataValue() { X = 0, Y = this.PyIV };
    //            case "PzIV":
    //                return new DataValue() { X = 0, Y = this.PzIV };
    //        }
    //        return null;
    //    }
    //    public float VxI;
    //    public float VyI;
    //    public float VzI;
    //    public float PxI;
    //    public float PyI;
    //    public float PzI;
    //    public float VxIV;
    //    public float VyIV;
    //    public float VzIV;
    //    public float PxIV;
    //    public float PyIV;
    //    public float PzIV;

    //    public static LogEntryEST4 Read(BinaryReader reader)
    //    {
    //        LogEntryEST4 result = new LogEntryEST4();
    //        result.VxI = reader.ReadSingle();
    //        result.VyI = reader.ReadSingle();
    //        result.VzI = reader.ReadSingle();
    //        result.PxI = reader.ReadSingle();
    //        result.PyI = reader.ReadSingle();
    //        result.PzI = reader.ReadSingle();
    //        result.VxIV = reader.ReadSingle();
    //        result.VyIV = reader.ReadSingle();
    //        result.VzIV = reader.ReadSingle();
    //        result.PxIV = reader.ReadSingle();
    //        result.PyIV = reader.ReadSingle();
    //        result.PzIV = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT EST5, 43, ffffffffff, MAGxI,MAGyI,MAGzI,MAGxIV,MAGyIV,MAGzIV,HeadI,HeadIV,AirI,AirIV
    //class LogEntryEST5 : LogEntry
    //{
    //    public override string GetName() { return "EST5"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "MAGxI":
    //                return new DataValue() { X = 0, Y = this.MAGxI };
    //            case "MAGyI":
    //                return new DataValue() { X = 0, Y = this.MAGyI };
    //            case "MAGzI":
    //                return new DataValue() { X = 0, Y = this.MAGzI };
    //            case "MAGxIV":
    //                return new DataValue() { X = 0, Y = this.MAGxIV };
    //            case "MAGyIV":
    //                return new DataValue() { X = 0, Y = this.MAGyIV };
    //            case "MAGzIV":
    //                return new DataValue() { X = 0, Y = this.MAGzIV };
    //            case "HeadI":
    //                return new DataValue() { X = 0, Y = this.HeadI };
    //            case "HeadIV":
    //                return new DataValue() { X = 0, Y = this.HeadIV };
    //            case "AirI":
    //                return new DataValue() { X = 0, Y = this.AirI };
    //            case "AirIV":
    //                return new DataValue() { X = 0, Y = this.AirIV };
    //        }
    //        return null;
    //    }
    //    public float MAGxI;
    //    public float MAGyI;
    //    public float MAGzI;
    //    public float MAGxIV;
    //    public float MAGyIV;
    //    public float MAGzIV;
    //    public float HeadI;
    //    public float HeadIV;
    //    public float AirI;
    //    public float AirIV;

    //    public static LogEntryEST5 Read(BinaryReader reader)
    //    {
    //        LogEntryEST5 result = new LogEntryEST5();
    //        result.MAGxI = reader.ReadSingle();
    //        result.MAGyI = reader.ReadSingle();
    //        result.MAGzI = reader.ReadSingle();
    //        result.MAGxIV = reader.ReadSingle();
    //        result.MAGyIV = reader.ReadSingle();
    //        result.MAGzIV = reader.ReadSingle();
    //        result.HeadI = reader.ReadSingle();
    //        result.HeadIV = reader.ReadSingle();
    //        result.AirI = reader.ReadSingle();
    //        result.AirIV = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT EST6, 27, ffffff, FxI,FyI,FxIV,FyIV,HAGLI,HAGLIV
    //class LogEntryEST6 : LogEntry
    //{
    //    public override string GetName() { return "EST6"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "FxI":
    //                return new DataValue() { X = 0, Y = this.FxI };
    //            case "FyI":
    //                return new DataValue() { X = 0, Y = this.FyI };
    //            case "FxIV":
    //                return new DataValue() { X = 0, Y = this.FxIV };
    //            case "FyIV":
    //                return new DataValue() { X = 0, Y = this.FyIV };
    //            case "HAGLI":
    //                return new DataValue() { X = 0, Y = this.HAGLI };
    //            case "HAGLIV":
    //                return new DataValue() { X = 0, Y = this.HAGLIV };
    //        }
    //        return null;
    //    }
    //    public float FxI;
    //    public float FyI;
    //    public float FxIV;
    //    public float FyIV;
    //    public float HAGLI;
    //    public float HAGLIV;

    //    public static LogEntryEST6 Read(BinaryReader reader)
    //    {
    //        LogEntryEST6 result = new LogEntryEST6();
    //        result.FxI = reader.ReadSingle();
    //        result.FyI = reader.ReadSingle();
    //        result.FxIV = reader.ReadSingle();
    //        result.FyIV = reader.ReadSingle();
    //        result.HAGLI = reader.ReadSingle();
    //        result.HAGLIV = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT PWR, 20, fffBBBBB, Periph5V,Servo5V,RSSI,UsbOk,BrickOk,ServoOk,PeriphOC,HipwrOC
    //class LogEntryPWR : LogEntry
    //{
    //    public override string GetName() { return "PWR"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Periph5V":
    //                return new DataValue() { X = 0, Y = this.Periph5V };
    //            case "Servo5V":
    //                return new DataValue() { X = 0, Y = this.Servo5V };
    //            case "RSSI":
    //                return new DataValue() { X = 0, Y = this.RSSI };
    //            case "UsbOk":
    //                return new DataValue() { X = 0, Y = this.UsbOk };
    //            case "BrickOk":
    //                return new DataValue() { X = 0, Y = this.BrickOk };
    //            case "ServoOk":
    //                return new DataValue() { X = 0, Y = this.ServoOk };
    //            case "PeriphOC":
    //                return new DataValue() { X = 0, Y = this.PeriphOC };
    //            case "HipwrOC":
    //                return new DataValue() { X = 0, Y = this.HipwrOC };
    //        }
    //        return null;
    //    }
    //    public float Periph5V;
    //    public float Servo5V;
    //    public float RSSI;
    //    public byte UsbOk;
    //    public byte BrickOk;
    //    public byte ServoOk;
    //    public byte PeriphOC;
    //    public byte HipwrOC;

    //    public static LogEntryPWR Read(BinaryReader reader)
    //    {
    //        LogEntryPWR result = new LogEntryPWR();
    //        result.Periph5V = reader.ReadSingle();
    //        result.Servo5V = reader.ReadSingle();
    //        result.RSSI = reader.ReadSingle();
    //        result.UsbOk = reader.ReadByte();
    //        result.BrickOk = reader.ReadByte();
    //        result.ServoOk = reader.ReadByte();
    //        result.PeriphOC = reader.ReadByte();
    //        result.HipwrOC = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT MOCP, 31, fffffff, QuatW,QuatX,QuatY,QuatZ,X,Y,Z
    //class LogEntryMOCP : LogEntry
    //{
    //    public override string GetName() { return "MOCP"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "QuatW":
    //                return new DataValue() { X = 0, Y = this.QuatW };
    //            case "QuatX":
    //                return new DataValue() { X = 0, Y = this.QuatX };
    //            case "QuatY":
    //                return new DataValue() { X = 0, Y = this.QuatY };
    //            case "QuatZ":
    //                return new DataValue() { X = 0, Y = this.QuatZ };
    //            case "X":
    //                return new DataValue() { X = 0, Y = this.X };
    //            case "Y":
    //                return new DataValue() { X = 0, Y = this.Y };
    //            case "Z":
    //                return new DataValue() { X = 0, Y = this.Z };
    //        }
    //        return null;
    //    }
    //    public float QuatW;
    //    public float QuatX;
    //    public float QuatY;
    //    public float QuatZ;
    //    public float X;
    //    public float Y;
    //    public float Z;

    //    public static LogEntryMOCP Read(BinaryReader reader)
    //    {
    //        LogEntryMOCP result = new LogEntryMOCP();
    //        result.QuatW = reader.ReadSingle();
    //        result.QuatX = reader.ReadSingle();
    //        result.QuatY = reader.ReadSingle();
    //        result.QuatZ = reader.ReadSingle();
    //        result.X = reader.ReadSingle();
    //        result.Y = reader.ReadSingle();
    //        result.Z = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT VISN, 43, ffffffffff, X,Y,Z,VX,VY,VZ,QuatW,QuatX,QuatY,QuatZ
    //class LogEntryVISN : LogEntry
    //{
    //    public override string GetName() { return "VISN"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "X":
    //                return new DataValue() { X = 0, Y = this.X };
    //            case "Y":
    //                return new DataValue() { X = 0, Y = this.Y };
    //            case "Z":
    //                return new DataValue() { X = 0, Y = this.Z };
    //            case "VX":
    //                return new DataValue() { X = 0, Y = this.VX };
    //            case "VY":
    //                return new DataValue() { X = 0, Y = this.VY };
    //            case "VZ":
    //                return new DataValue() { X = 0, Y = this.VZ };
    //            case "QuatW":
    //                return new DataValue() { X = 0, Y = this.QuatW };
    //            case "QuatX":
    //                return new DataValue() { X = 0, Y = this.QuatX };
    //            case "QuatY":
    //                return new DataValue() { X = 0, Y = this.QuatY };
    //            case "QuatZ":
    //                return new DataValue() { X = 0, Y = this.QuatZ };
    //        }
    //        return null;
    //    }
    //    public float X;
    //    public float Y;
    //    public float Z;
    //    public float VX;
    //    public float VY;
    //    public float VZ;
    //    public float QuatW;
    //    public float QuatX;
    //    public float QuatY;
    //    public float QuatZ;

    //    public static LogEntryVISN Read(BinaryReader reader)
    //    {
    //        LogEntryVISN result = new LogEntryVISN();
    //        result.X = reader.ReadSingle();
    //        result.Y = reader.ReadSingle();
    //        result.Z = reader.ReadSingle();
    //        result.VX = reader.ReadSingle();
    //        result.VY = reader.ReadSingle();
    //        result.VZ = reader.ReadSingle();
    //        result.QuatW = reader.ReadSingle();
    //        result.QuatX = reader.ReadSingle();
    //        result.QuatY = reader.ReadSingle();
    //        result.QuatZ = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT GS0A, 19, BBBBBBBBBBBBBBBB, s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15
    //class LogEntryGS0A : LogEntry
    //{
    //    public override string GetName() { return "GS0A"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "s0":
    //                return new DataValue() { X = 0, Y = this.s0 };
    //            case "s1":
    //                return new DataValue() { X = 0, Y = this.s1 };
    //            case "s2":
    //                return new DataValue() { X = 0, Y = this.s2 };
    //            case "s3":
    //                return new DataValue() { X = 0, Y = this.s3 };
    //            case "s4":
    //                return new DataValue() { X = 0, Y = this.s4 };
    //            case "s5":
    //                return new DataValue() { X = 0, Y = this.s5 };
    //            case "s6":
    //                return new DataValue() { X = 0, Y = this.s6 };
    //            case "s7":
    //                return new DataValue() { X = 0, Y = this.s7 };
    //            case "s8":
    //                return new DataValue() { X = 0, Y = this.s8 };
    //            case "s9":
    //                return new DataValue() { X = 0, Y = this.s9 };
    //            case "s10":
    //                return new DataValue() { X = 0, Y = this.s10 };
    //            case "s11":
    //                return new DataValue() { X = 0, Y = this.s11 };
    //            case "s12":
    //                return new DataValue() { X = 0, Y = this.s12 };
    //            case "s13":
    //                return new DataValue() { X = 0, Y = this.s13 };
    //            case "s14":
    //                return new DataValue() { X = 0, Y = this.s14 };
    //            case "s15":
    //                return new DataValue() { X = 0, Y = this.s15 };
    //        }
    //        return null;
    //    }
    //    public byte s0;
    //    public byte s1;
    //    public byte s2;
    //    public byte s3;
    //    public byte s4;
    //    public byte s5;
    //    public byte s6;
    //    public byte s7;
    //    public byte s8;
    //    public byte s9;
    //    public byte s10;
    //    public byte s11;
    //    public byte s12;
    //    public byte s13;
    //    public byte s14;
    //    public byte s15;

    //    public static LogEntryGS0A Read(BinaryReader reader)
    //    {
    //        LogEntryGS0A result = new LogEntryGS0A();
    //        result.s0 = reader.ReadByte();
    //        result.s1 = reader.ReadByte();
    //        result.s2 = reader.ReadByte();
    //        result.s3 = reader.ReadByte();
    //        result.s4 = reader.ReadByte();
    //        result.s5 = reader.ReadByte();
    //        result.s6 = reader.ReadByte();
    //        result.s7 = reader.ReadByte();
    //        result.s8 = reader.ReadByte();
    //        result.s9 = reader.ReadByte();
    //        result.s10 = reader.ReadByte();
    //        result.s11 = reader.ReadByte();
    //        result.s12 = reader.ReadByte();
    //        result.s13 = reader.ReadByte();
    //        result.s14 = reader.ReadByte();
    //        result.s15 = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT GS0B, 19, BBBBBBBBBBBBBBBB, s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15
    //class LogEntryGS0B : LogEntry
    //{
    //    public override string GetName() { return "GS0B"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "s0":
    //                return new DataValue() { X = 0, Y = this.s0 };
    //            case "s1":
    //                return new DataValue() { X = 0, Y = this.s1 };
    //            case "s2":
    //                return new DataValue() { X = 0, Y = this.s2 };
    //            case "s3":
    //                return new DataValue() { X = 0, Y = this.s3 };
    //            case "s4":
    //                return new DataValue() { X = 0, Y = this.s4 };
    //            case "s5":
    //                return new DataValue() { X = 0, Y = this.s5 };
    //            case "s6":
    //                return new DataValue() { X = 0, Y = this.s6 };
    //            case "s7":
    //                return new DataValue() { X = 0, Y = this.s7 };
    //            case "s8":
    //                return new DataValue() { X = 0, Y = this.s8 };
    //            case "s9":
    //                return new DataValue() { X = 0, Y = this.s9 };
    //            case "s10":
    //                return new DataValue() { X = 0, Y = this.s10 };
    //            case "s11":
    //                return new DataValue() { X = 0, Y = this.s11 };
    //            case "s12":
    //                return new DataValue() { X = 0, Y = this.s12 };
    //            case "s13":
    //                return new DataValue() { X = 0, Y = this.s13 };
    //            case "s14":
    //                return new DataValue() { X = 0, Y = this.s14 };
    //            case "s15":
    //                return new DataValue() { X = 0, Y = this.s15 };
    //        }
    //        return null;
    //    }
    //    public byte s0;
    //    public byte s1;
    //    public byte s2;
    //    public byte s3;
    //    public byte s4;
    //    public byte s5;
    //    public byte s6;
    //    public byte s7;
    //    public byte s8;
    //    public byte s9;
    //    public byte s10;
    //    public byte s11;
    //    public byte s12;
    //    public byte s13;
    //    public byte s14;
    //    public byte s15;

    //    public static LogEntryGS0B Read(BinaryReader reader)
    //    {
    //        LogEntryGS0B result = new LogEntryGS0B();
    //        result.s0 = reader.ReadByte();
    //        result.s1 = reader.ReadByte();
    //        result.s2 = reader.ReadByte();
    //        result.s3 = reader.ReadByte();
    //        result.s4 = reader.ReadByte();
    //        result.s5 = reader.ReadByte();
    //        result.s6 = reader.ReadByte();
    //        result.s7 = reader.ReadByte();
    //        result.s8 = reader.ReadByte();
    //        result.s9 = reader.ReadByte();
    //        result.s10 = reader.ReadByte();
    //        result.s11 = reader.ReadByte();
    //        result.s12 = reader.ReadByte();
    //        result.s13 = reader.ReadByte();
    //        result.s14 = reader.ReadByte();
    //        result.s15 = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT GS1A, 19, BBBBBBBBBBBBBBBB, s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15
    //class LogEntryGS1A : LogEntry
    //{
    //    public override string GetName() { return "GS1A"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "s0":
    //                return new DataValue() { X = 0, Y = this.s0 };
    //            case "s1":
    //                return new DataValue() { X = 0, Y = this.s1 };
    //            case "s2":
    //                return new DataValue() { X = 0, Y = this.s2 };
    //            case "s3":
    //                return new DataValue() { X = 0, Y = this.s3 };
    //            case "s4":
    //                return new DataValue() { X = 0, Y = this.s4 };
    //            case "s5":
    //                return new DataValue() { X = 0, Y = this.s5 };
    //            case "s6":
    //                return new DataValue() { X = 0, Y = this.s6 };
    //            case "s7":
    //                return new DataValue() { X = 0, Y = this.s7 };
    //            case "s8":
    //                return new DataValue() { X = 0, Y = this.s8 };
    //            case "s9":
    //                return new DataValue() { X = 0, Y = this.s9 };
    //            case "s10":
    //                return new DataValue() { X = 0, Y = this.s10 };
    //            case "s11":
    //                return new DataValue() { X = 0, Y = this.s11 };
    //            case "s12":
    //                return new DataValue() { X = 0, Y = this.s12 };
    //            case "s13":
    //                return new DataValue() { X = 0, Y = this.s13 };
    //            case "s14":
    //                return new DataValue() { X = 0, Y = this.s14 };
    //            case "s15":
    //                return new DataValue() { X = 0, Y = this.s15 };
    //        }
    //        return null;
    //    }
    //    public byte s0;
    //    public byte s1;
    //    public byte s2;
    //    public byte s3;
    //    public byte s4;
    //    public byte s5;
    //    public byte s6;
    //    public byte s7;
    //    public byte s8;
    //    public byte s9;
    //    public byte s10;
    //    public byte s11;
    //    public byte s12;
    //    public byte s13;
    //    public byte s14;
    //    public byte s15;

    //    public static LogEntryGS1A Read(BinaryReader reader)
    //    {
    //        LogEntryGS1A result = new LogEntryGS1A();
    //        result.s0 = reader.ReadByte();
    //        result.s1 = reader.ReadByte();
    //        result.s2 = reader.ReadByte();
    //        result.s3 = reader.ReadByte();
    //        result.s4 = reader.ReadByte();
    //        result.s5 = reader.ReadByte();
    //        result.s6 = reader.ReadByte();
    //        result.s7 = reader.ReadByte();
    //        result.s8 = reader.ReadByte();
    //        result.s9 = reader.ReadByte();
    //        result.s10 = reader.ReadByte();
    //        result.s11 = reader.ReadByte();
    //        result.s12 = reader.ReadByte();
    //        result.s13 = reader.ReadByte();
    //        result.s14 = reader.ReadByte();
    //        result.s15 = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT GS1B, 19, BBBBBBBBBBBBBBBB, s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15
    //class LogEntryGS1B : LogEntry
    //{
    //    public override string GetName() { return "GS1B"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "s0":
    //                return new DataValue() { X = 0, Y = this.s0 };
    //            case "s1":
    //                return new DataValue() { X = 0, Y = this.s1 };
    //            case "s2":
    //                return new DataValue() { X = 0, Y = this.s2 };
    //            case "s3":
    //                return new DataValue() { X = 0, Y = this.s3 };
    //            case "s4":
    //                return new DataValue() { X = 0, Y = this.s4 };
    //            case "s5":
    //                return new DataValue() { X = 0, Y = this.s5 };
    //            case "s6":
    //                return new DataValue() { X = 0, Y = this.s6 };
    //            case "s7":
    //                return new DataValue() { X = 0, Y = this.s7 };
    //            case "s8":
    //                return new DataValue() { X = 0, Y = this.s8 };
    //            case "s9":
    //                return new DataValue() { X = 0, Y = this.s9 };
    //            case "s10":
    //                return new DataValue() { X = 0, Y = this.s10 };
    //            case "s11":
    //                return new DataValue() { X = 0, Y = this.s11 };
    //            case "s12":
    //                return new DataValue() { X = 0, Y = this.s12 };
    //            case "s13":
    //                return new DataValue() { X = 0, Y = this.s13 };
    //            case "s14":
    //                return new DataValue() { X = 0, Y = this.s14 };
    //            case "s15":
    //                return new DataValue() { X = 0, Y = this.s15 };
    //        }
    //        return null;
    //    }
    //    public byte s0;
    //    public byte s1;
    //    public byte s2;
    //    public byte s3;
    //    public byte s4;
    //    public byte s5;
    //    public byte s6;
    //    public byte s7;
    //    public byte s8;
    //    public byte s9;
    //    public byte s10;
    //    public byte s11;
    //    public byte s12;
    //    public byte s13;
    //    public byte s14;
    //    public byte s15;

    //    public static LogEntryGS1B Read(BinaryReader reader)
    //    {
    //        LogEntryGS1B result = new LogEntryGS1B();
    //        result.s0 = reader.ReadByte();
    //        result.s1 = reader.ReadByte();
    //        result.s2 = reader.ReadByte();
    //        result.s3 = reader.ReadByte();
    //        result.s4 = reader.ReadByte();
    //        result.s5 = reader.ReadByte();
    //        result.s6 = reader.ReadByte();
    //        result.s7 = reader.ReadByte();
    //        result.s8 = reader.ReadByte();
    //        result.s9 = reader.ReadByte();
    //        result.s10 = reader.ReadByte();
    //        result.s11 = reader.ReadByte();
    //        result.s12 = reader.ReadByte();
    //        result.s13 = reader.ReadByte();
    //        result.s14 = reader.ReadByte();
    //        result.s15 = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT TECS, 60, ffffffffffffffB, ASP,AF,FSP,F,AsSP,AsF,AsDSP,AsD,EE,ERE,EDE,EDRE,PtchI,ThrI,M
    //class LogEntryTECS : LogEntry
    //{
    //    public override string GetName() { return "TECS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "ASP":
    //                return new DataValue() { X = 0, Y = this.ASP };
    //            case "AF":
    //                return new DataValue() { X = 0, Y = this.AF };
    //            case "FSP":
    //                return new DataValue() { X = 0, Y = this.FSP };
    //            case "F":
    //                return new DataValue() { X = 0, Y = this.F };
    //            case "AsSP":
    //                return new DataValue() { X = 0, Y = this.AsSP };
    //            case "AsF":
    //                return new DataValue() { X = 0, Y = this.AsF };
    //            case "AsDSP":
    //                return new DataValue() { X = 0, Y = this.AsDSP };
    //            case "AsD":
    //                return new DataValue() { X = 0, Y = this.AsD };
    //            case "EE":
    //                return new DataValue() { X = 0, Y = this.EE };
    //            case "ERE":
    //                return new DataValue() { X = 0, Y = this.ERE };
    //            case "EDE":
    //                return new DataValue() { X = 0, Y = this.EDE };
    //            case "EDRE":
    //                return new DataValue() { X = 0, Y = this.EDRE };
    //            case "PtchI":
    //                return new DataValue() { X = 0, Y = this.PtchI };
    //            case "ThrI":
    //                return new DataValue() { X = 0, Y = this.ThrI };
    //            case "M":
    //                return new DataValue() { X = 0, Y = this.M };
    //        }
    //        return null;
    //    }
    //    public float ASP;
    //    public float AF;
    //    public float FSP;
    //    public float F;
    //    public float AsSP;
    //    public float AsF;
    //    public float AsDSP;
    //    public float AsD;
    //    public float EE;
    //    public float ERE;
    //    public float EDE;
    //    public float EDRE;
    //    public float PtchI;
    //    public float ThrI;
    //    public byte M;

    //    public static LogEntryTECS Read(BinaryReader reader)
    //    {
    //        LogEntryTECS result = new LogEntryTECS();
    //        result.ASP = reader.ReadSingle();
    //        result.AF = reader.ReadSingle();
    //        result.FSP = reader.ReadSingle();
    //        result.F = reader.ReadSingle();
    //        result.AsSP = reader.ReadSingle();
    //        result.AsF = reader.ReadSingle();
    //        result.AsDSP = reader.ReadSingle();
    //        result.AsD = reader.ReadSingle();
    //        result.EE = reader.ReadSingle();
    //        result.ERE = reader.ReadSingle();
    //        result.EDE = reader.ReadSingle();
    //        result.EDRE = reader.ReadSingle();
    //        result.PtchI = reader.ReadSingle();
    //        result.ThrI = reader.ReadSingle();
    //        result.M = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT WIND, 19, ffff, X,Y,CovX,CovY
    //class LogEntryWIND : LogEntry
    //{
    //    public override string GetName() { return "WIND"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "X":
    //                return new DataValue() { X = 0, Y = this.X };
    //            case "Y":
    //                return new DataValue() { X = 0, Y = this.Y };
    //            case "CovX":
    //                return new DataValue() { X = 0, Y = this.CovX };
    //            case "CovY":
    //                return new DataValue() { X = 0, Y = this.CovY };
    //        }
    //        return null;
    //    }
    //    public float X;
    //    public float Y;
    //    public float CovX;
    //    public float CovY;

    //    public static LogEntryWIND Read(BinaryReader reader)
    //    {
    //        LogEntryWIND result = new LogEntryWIND();
    //        result.X = reader.ReadSingle();
    //        result.Y = reader.ReadSingle();
    //        result.CovX = reader.ReadSingle();
    //        result.CovY = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT ENCD, 27, qfqf, cnt0,vel0,cnt1,vel1
    //class LogEntryENCD : LogEntry
    //{
    //    public override string GetName() { return "ENCD"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "cnt0":
    //                return new DataValue() { X = 0, Y = this.cnt0 };
    //            case "vel0":
    //                return new DataValue() { X = 0, Y = this.vel0 };
    //            case "cnt1":
    //                return new DataValue() { X = 0, Y = this.cnt1 };
    //            case "vel1":
    //                return new DataValue() { X = 0, Y = this.vel1 };
    //        }
    //        return null;
    //    }
    //    public Int64 cnt0;
    //    public float vel0;
    //    public Int64 cnt1;
    //    public float vel1;

    //    public static LogEntryENCD Read(BinaryReader reader)
    //    {
    //        LogEntryENCD result = new LogEntryENCD();
    //        result.cnt0 = reader.ReadInt64();
    //        result.vel0 = reader.ReadSingle();
    //        result.cnt1 = reader.ReadInt64();
    //        result.vel1 = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT TSYN, 11, Q, TimeOffset
    //class LogEntryTSYN : LogEntry
    //{
    //    public override string GetName() { return "TSYN"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "TimeOffset":
    //                return new DataValue() { X = 0, Y = this.TimeOffset };
    //        }
    //        return null;
    //    }
    //    public UInt64 TimeOffset;

    //    public static LogEntryTSYN Read(BinaryReader reader)
    //    {
    //        LogEntryTSYN result = new LogEntryTSYN();
    //        result.TimeOffset = reader.ReadUInt64();
    //        return result;
    //    }
    //}

    //// FMT MACS, 15, fff, RRint,PRint,YRint
    //class LogEntryMACS : LogEntry
    //{
    //    public override string GetName() { return "MACS"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "RRint":
    //                return new DataValue() { X = 0, Y = this.RRint };
    //            case "PRint":
    //                return new DataValue() { X = 0, Y = this.PRint };
    //            case "YRint":
    //                return new DataValue() { X = 0, Y = this.YRint };
    //        }
    //        return null;
    //    }
    //    public float RRint;
    //    public float PRint;
    //    public float YRint;

    //    public static LogEntryMACS Read(BinaryReader reader)
    //    {
    //        LogEntryMACS result = new LogEntryMACS();
    //        result.RRint = reader.ReadSingle();
    //        result.PRint = reader.ReadSingle();
    //        result.YRint = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT CAMT, 15, QI, timestamp,seq
    //class LogEntryCAMT : LogEntry
    //{
    //    public override string GetName() { return "CAMT"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "timestamp":
    //                return new DataValue() { X = 0, Y = this.timestamp };
    //            case "seq":
    //                return new DataValue() { X = 0, Y = this.seq };
    //        }
    //        return null;
    //    }
    //    public UInt64 timestamp;
    //    public UInt32 seq;

    //    public static LogEntryCAMT Read(BinaryReader reader)
    //    {
    //        LogEntryCAMT result = new LogEntryCAMT();
    //        result.timestamp = reader.ReadUInt64();
    //        result.seq = reader.ReadUInt32();
    //        return result;
    //    }
    //}

    //// FMT RPL1, 75, QffQQffffffffff, t,gIdt,aIdt,Tm,Tb,gx,gy,gz,ax,ay,az,magX,magY,magZ,b_alt
    //class LogEntryRPL1 : LogEntry
    //{
    //    public override string GetName() { return "RPL1"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "t":
    //                return new DataValue() { X = 0, Y = this.t };
    //            case "gIdt":
    //                return new DataValue() { X = 0, Y = this.gIdt };
    //            case "aIdt":
    //                return new DataValue() { X = 0, Y = this.aIdt };
    //            case "Tm":
    //                return new DataValue() { X = 0, Y = this.Tm };
    //            case "Tb":
    //                return new DataValue() { X = 0, Y = this.Tb };
    //            case "gx":
    //                return new DataValue() { X = 0, Y = this.gx };
    //            case "gy":
    //                return new DataValue() { X = 0, Y = this.gy };
    //            case "gz":
    //                return new DataValue() { X = 0, Y = this.gz };
    //            case "ax":
    //                return new DataValue() { X = 0, Y = this.ax };
    //            case "ay":
    //                return new DataValue() { X = 0, Y = this.ay };
    //            case "az":
    //                return new DataValue() { X = 0, Y = this.az };
    //            case "magX":
    //                return new DataValue() { X = 0, Y = this.magX };
    //            case "magY":
    //                return new DataValue() { X = 0, Y = this.magY };
    //            case "magZ":
    //                return new DataValue() { X = 0, Y = this.magZ };
    //            case "b_alt":
    //                return new DataValue() { X = 0, Y = this.b_alt };
    //        }
    //        return null;
    //    }
    //    public UInt64 t;
    //    public float gIdt;
    //    public float aIdt;
    //    public UInt64 Tm;
    //    public UInt64 Tb;
    //    public float gx;
    //    public float gy;
    //    public float gz;
    //    public float ax;
    //    public float ay;
    //    public float az;
    //    public float magX;
    //    public float magY;
    //    public float magZ;
    //    public float b_alt;

    //    public static LogEntryRPL1 Read(BinaryReader reader)
    //    {
    //        LogEntryRPL1 result = new LogEntryRPL1();
    //        result.t = reader.ReadUInt64();
    //        result.gIdt = reader.ReadSingle();
    //        result.aIdt = reader.ReadSingle();
    //        result.Tm = reader.ReadUInt64();
    //        result.Tb = reader.ReadUInt64();
    //        result.gx = reader.ReadSingle();
    //        result.gy = reader.ReadSingle();
    //        result.gz = reader.ReadSingle();
    //        result.ax = reader.ReadSingle();
    //        result.ay = reader.ReadSingle();
    //        result.az = reader.ReadSingle();
    //        result.magX = reader.ReadSingle();
    //        result.magY = reader.ReadSingle();
    //        result.magZ = reader.ReadSingle();
    //        result.b_alt = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT RPL2, 62, QQLLiMMfffffffM, Tpos,Tvel,lat,lon,alt,fix,nsats,eph,epv,sacc,v,vN,vE,vD,v_val
    //class LogEntryRPL2 : LogEntry
    //{
    //    public override string GetName() { return "RPL2"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Tpos":
    //                return new DataValue() { X = 0, Y = this.Tpos };
    //            case "Tvel":
    //                return new DataValue() { X = 0, Y = this.Tvel };
    //            case "lat":
    //                return new DataValue() { X = 0, Y = this.lat };
    //            case "lon":
    //                return new DataValue() { X = 0, Y = this.lon };
    //            case "alt":
    //                return new DataValue() { X = 0, Y = this.alt };
    //            case "fix":
    //                return new DataValue() { X = 0, Y = this.fix };
    //            case "nsats":
    //                return new DataValue() { X = 0, Y = this.nsats };
    //            case "eph":
    //                return new DataValue() { X = 0, Y = this.eph };
    //            case "epv":
    //                return new DataValue() { X = 0, Y = this.epv };
    //            case "sacc":
    //                return new DataValue() { X = 0, Y = this.sacc };
    //            case "v":
    //                return new DataValue() { X = 0, Y = this.v };
    //            case "vN":
    //                return new DataValue() { X = 0, Y = this.vN };
    //            case "vE":
    //                return new DataValue() { X = 0, Y = this.vE };
    //            case "vD":
    //                return new DataValue() { X = 0, Y = this.vD };
    //            case "v_val":
    //                return new DataValue() { X = 0, Y = this.v_val };
    //        }
    //        return null;
    //    }
    //    public UInt64 Tpos;
    //    public UInt64 Tvel;
    //    public double lat;
    //    public double lon;
    //    public Int32 alt;
    //    public byte fix;
    //    public byte nsats;
    //    public float eph;
    //    public float epv;
    //    public float sacc;
    //    public float v;
    //    public float vN;
    //    public float vE;
    //    public float vD;
    //    public byte v_val;

    //    public static LogEntryRPL2 Read(BinaryReader reader)
    //    {
    //        LogEntryRPL2 result = new LogEntryRPL2();
    //        result.Tpos = reader.ReadUInt64();
    //        result.Tvel = reader.ReadUInt64();
    //        result.lat = reader.ReadInt32() / 10000000.0;
    //        result.lon = reader.ReadInt32() / 10000000.0;
    //        result.alt = reader.ReadInt32();
    //        result.fix = reader.ReadByte();
    //        result.nsats = reader.ReadByte();
    //        result.eph = reader.ReadSingle();
    //        result.epv = reader.ReadSingle();
    //        result.sacc = reader.ReadSingle();
    //        result.v = reader.ReadSingle();
    //        result.vN = reader.ReadSingle();
    //        result.vE = reader.ReadSingle();
    //        result.vD = reader.ReadSingle();
    //        result.v_val = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT RPL3, 32, QffffIB, Tflow,fx,fy,gx,gy,delT,qual
    //class LogEntryRPL3 : LogEntry
    //{
    //    public override string GetName() { return "RPL3"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Tflow":
    //                return new DataValue() { X = 0, Y = this.Tflow };
    //            case "fx":
    //                return new DataValue() { X = 0, Y = this.fx };
    //            case "fy":
    //                return new DataValue() { X = 0, Y = this.fy };
    //            case "gx":
    //                return new DataValue() { X = 0, Y = this.gx };
    //            case "gy":
    //                return new DataValue() { X = 0, Y = this.gy };
    //            case "delT":
    //                return new DataValue() { X = 0, Y = this.delT };
    //            case "qual":
    //                return new DataValue() { X = 0, Y = this.qual };
    //        }
    //        return null;
    //    }
    //    public UInt64 Tflow;
    //    public float fx;
    //    public float fy;
    //    public float gx;
    //    public float gy;
    //    public UInt32 delT;
    //    public byte qual;

    //    public static LogEntryRPL3 Read(BinaryReader reader)
    //    {
    //        LogEntryRPL3 result = new LogEntryRPL3();
    //        result.Tflow = reader.ReadUInt64();
    //        result.fx = reader.ReadSingle();
    //        result.fy = reader.ReadSingle();
    //        result.gx = reader.ReadSingle();
    //        result.gy = reader.ReadSingle();
    //        result.delT = reader.ReadUInt32();
    //        result.qual = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT RPL4, 15, Qf, Trng,rng
    //class LogEntryRPL4 : LogEntry
    //{
    //    public override string GetName() { return "RPL4"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Trng":
    //                return new DataValue() { X = 0, Y = this.Trng };
    //            case "rng":
    //                return new DataValue() { X = 0, Y = this.rng };
    //        }
    //        return null;
    //    }
    //    public UInt64 Trng;
    //    public float rng;

    //    public static LogEntryRPL4 Read(BinaryReader reader)
    //    {
    //        LogEntryRPL4 result = new LogEntryRPL4();
    //        result.Trng = reader.ReadUInt64();
    //        result.rng = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT RPL5, 47, Qfffffffff, Tev,x,y,z,q0,q1,q2,q3,posErr,angErr
    //class LogEntryRPL5 : LogEntry
    //{
    //    public override string GetName() { return "RPL5"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Tev":
    //                return new DataValue() { X = 0, Y = this.Tev };
    //            case "x":
    //                return new DataValue() { X = 0, Y = this.x };
    //            case "y":
    //                return new DataValue() { X = 0, Y = this.y };
    //            case "z":
    //                return new DataValue() { X = 0, Y = this.z };
    //            case "q0":
    //                return new DataValue() { X = 0, Y = this.q0 };
    //            case "q1":
    //                return new DataValue() { X = 0, Y = this.q1 };
    //            case "q2":
    //                return new DataValue() { X = 0, Y = this.q2 };
    //            case "q3":
    //                return new DataValue() { X = 0, Y = this.q3 };
    //            case "posErr":
    //                return new DataValue() { X = 0, Y = this.posErr };
    //            case "angErr":
    //                return new DataValue() { X = 0, Y = this.angErr };
    //        }
    //        return null;
    //    }
    //    public UInt64 Tev;
    //    public float x;
    //    public float y;
    //    public float z;
    //    public float q0;
    //    public float q1;
    //    public float q2;
    //    public float q3;
    //    public float posErr;
    //    public float angErr;

    //    public static LogEntryRPL5 Read(BinaryReader reader)
    //    {
    //        LogEntryRPL5 result = new LogEntryRPL5();
    //        result.Tev = reader.ReadUInt64();
    //        result.x = reader.ReadSingle();
    //        result.y = reader.ReadSingle();
    //        result.z = reader.ReadSingle();
    //        result.q0 = reader.ReadSingle();
    //        result.q1 = reader.ReadSingle();
    //        result.q2 = reader.ReadSingle();
    //        result.q3 = reader.ReadSingle();
    //        result.posErr = reader.ReadSingle();
    //        result.angErr = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT RPL6, 19, Qff, Tasp,inAsp,trAsp
    //class LogEntryRPL6 : LogEntry
    //{
    //    public override string GetName() { return "RPL6"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Tasp":
    //                return new DataValue() { X = 0, Y = this.Tasp };
    //            case "inAsp":
    //                return new DataValue() { X = 0, Y = this.inAsp };
    //            case "trAsp":
    //                return new DataValue() { X = 0, Y = this.trAsp };
    //        }
    //        return null;
    //    }
    //    public UInt64 Tasp;
    //    public float inAsp;
    //    public float trAsp;

    //    public static LogEntryRPL6 Read(BinaryReader reader)
    //    {
    //        LogEntryRPL6 result = new LogEntryRPL6();
    //        result.Tasp = reader.ReadUInt64();
    //        result.inAsp = reader.ReadSingle();
    //        result.trAsp = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT LAND, 4, B, Landed
    //class LogEntryLAND : LogEntry
    //{
    //    public override string GetName() { return "LAND"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Landed":
    //                return new DataValue() { X = 0, Y = this.Landed };
    //        }
    //        return null;
    //    }
    //    public byte Landed;

    //    public static LogEntryLAND Read(BinaryReader reader)
    //    {
    //        LogEntryLAND result = new LogEntryLAND();
    //        result.Landed = reader.ReadByte();
    //        return result;
    //    }
    //}

    //// FMT LOAD, 7, f, CPU
    //class LogEntryLOAD : LogEntry
    //{
    //    public override string GetName() { return "LOAD"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "CPU":
    //                return new DataValue() { X = 0, Y = this.CPU };
    //        }
    //        return null;
    //    }
    //    public float CPU;

    //    public static LogEntryLOAD Read(BinaryReader reader)
    //    {
    //        LogEntryLOAD result = new LogEntryLOAD();
    //        result.CPU = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //// FMT TIME, 11, Q, StartTime
    //class LogEntryTIME : LogEntry
    //{
    //    public override string GetName() { return "TIME"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "StartTime":
    //                return new DataValue() { X = 0, Y = this.StartTime };
    //        }
    //        return null;
    //    }
    //    public UInt64 StartTime;

    //    public static LogEntryTIME Read(BinaryReader reader)
    //    {
    //        LogEntryTIME result = new LogEntryTIME();
    //        result.StartTime = reader.ReadUInt64();
    //        return result;
    //    }
    //}

    //// FMT VER, 83, NZ, Arch,FwGit
    //class LogEntryVER : LogEntry
    //{
    //    public override string GetName() { return "VER"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Arch":
    //                return new DataValue() { X = 0, Y = 0 };
    //            case "FwGit":
    //                return new DataValue() { X = 0, Y = 0 };
    //        }
    //        return null;
    //    }
    //    public string Arch;
    //    public string FwGit;

    //    public static LogEntryVER Read(BinaryReader reader)
    //    {
    //        LogEntryVER result = new LogEntryVER();
    //        result.Arch = ReadAsciiString(reader, 16);
    //        result.FwGit = ReadAsciiString(reader, 64);
    //        return result;
    //    }
    //}

    //// FMT PARM, 23, Nf, Name,Value
    //class LogEntryPARM : LogEntry
    //{
    //    public override string GetName() { return "PARM"; }
    //    public override DataValue GetDataValue(string field)
    //    {
    //        switch (field)
    //        {
    //            case "Name":
    //                return new DataValue() { X = 0, Y = 0 };
    //            case "Value":
    //                return new DataValue() { X = 0, Y = this.Value };
    //        }
    //        return null;
    //    }
    //    public string Name;
    //    public float Value;

    //    public static LogEntryPARM Read(BinaryReader reader)
    //    {
    //        LogEntryPARM result = new LogEntryPARM();
    //        result.Name = ReadAsciiString(reader, 16);
    //        result.Value = reader.ReadSingle();
    //        return result;
    //    }
    //}

    //class LogEntryParser
    //{
    //    public static object ParseLogEntry(byte type, BinaryReader reader)
    //    {
    //        switch (type)
    //        {
    //            case 2:
    //                return LogEntryATT.Read(reader);
    //            case 3:
    //                return LogEntryATSP.Read(reader);
    //            case 4:
    //                return LogEntryIMU.Read(reader);
    //            case 22:
    //                return LogEntryIMU1.Read(reader);
    //            case 23:
    //                return LogEntryIMU2.Read(reader);
    //            case 5:
    //                return LogEntrySENS.Read(reader);
    //            case 42:
    //                return LogEntryAIR1.Read(reader);
    //            case 6:
    //                return LogEntryLPOS.Read(reader);
    //            case 7:
    //                return LogEntryLPSP.Read(reader);
    //            case 8:
    //                return LogEntryGPS.Read(reader);
    //            case 58:
    //                return LogEntryDGPS.Read(reader);
    //            case 9:
    //                return LogEntryATTC.Read(reader);
    //            case 46:
    //                return LogEntryATC1.Read(reader);
    //            case 10:
    //                return LogEntrySTAT.Read(reader);
    //            case 43:
    //                return LogEntryVTOL.Read(reader);
    //            case 47:
    //                return LogEntryCTS.Read(reader);
    //            case 11:
    //                return LogEntryRC.Read(reader);
    //            case 12:
    //                return LogEntryOUT0.Read(reader);
    //            case 50:
    //                return LogEntryOUT1.Read(reader);
    //            case 13:
    //                return LogEntryAIRS.Read(reader);
    //            case 14:
    //                return LogEntryARSP.Read(reader);
    //            case 15:
    //                return LogEntryFLOW.Read(reader);
    //            case 16:
    //                return LogEntryGPOS.Read(reader);
    //            case 17:
    //                return LogEntryGPSP.Read(reader);
    //            case 18:
    //                return LogEntryESC.Read(reader);
    //            case 19:
    //                return LogEntryGVSP.Read(reader);
    //            case 20:
    //                return LogEntryBATT.Read(reader);
    //            case 21:
    //                return LogEntryDIST.Read(reader);
    //            case 36:
    //                return LogEntryTEL0.Read(reader);
    //            case 37:
    //                return LogEntryTEL1.Read(reader);
    //            case 38:
    //                return LogEntryTEL2.Read(reader);
    //            case 39:
    //                return LogEntryTEL3.Read(reader);
    //            case 32:
    //                return LogEntryEST0.Read(reader);
    //            case 33:
    //                return LogEntryEST1.Read(reader);
    //            case 34:
    //                return LogEntryEST2.Read(reader);
    //            case 35:
    //                return LogEntryEST3.Read(reader);
    //            case 48:
    //                return LogEntryEST4.Read(reader);
    //            case 49:
    //                return LogEntryEST5.Read(reader);
    //            case 53:
    //                return LogEntryEST6.Read(reader);
    //            case 24:
    //                return LogEntryPWR.Read(reader);
    //            case 25:
    //                return LogEntryMOCP.Read(reader);
    //            case 40:
    //                return LogEntryVISN.Read(reader);
    //            case 26:
    //                return LogEntryGS0A.Read(reader);
    //            case 27:
    //                return LogEntryGS0B.Read(reader);
    //            case 28:
    //                return LogEntryGS1A.Read(reader);
    //            case 29:
    //                return LogEntryGS1B.Read(reader);
    //            case 30:
    //                return LogEntryTECS.Read(reader);
    //            case 31:
    //                return LogEntryWIND.Read(reader);
    //            case 41:
    //                return LogEntryENCD.Read(reader);
    //            case 44:
    //                return LogEntryTSYN.Read(reader);
    //            case 45:
    //                return LogEntryMACS.Read(reader);
    //            case 55:
    //                return LogEntryCAMT.Read(reader);
    //            case 51:
    //                return LogEntryRPL1.Read(reader);
    //            case 52:
    //                return LogEntryRPL2.Read(reader);
    //            case 54:
    //                return LogEntryRPL3.Read(reader);
    //            case 56:
    //                return LogEntryRPL4.Read(reader);
    //            case 60:
    //                return LogEntryRPL5.Read(reader);
    //            case 59:
    //                return LogEntryRPL6.Read(reader);
    //            case 57:
    //                return LogEntryLAND.Read(reader);
    //            case 61:
    //                return LogEntryLOAD.Read(reader);
    //            case 129:
    //                return LogEntryTIME.Read(reader);
    //            case 130:
    //                return LogEntryVER.Read(reader);
    //            case 131:
    //                return LogEntryPARM.Read(reader);
    //        }
    //        return null;
    //    }
    //}

}
