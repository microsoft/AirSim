//#define USE_SharpZipLib
/* * * * *
 * This is an extension of the SimpleJSON framework to provide methods to
 * serialize a JSON object tree into a compact binary format. Optionally the
 * binary stream can be compressed with the SharpZipLib when using the define
 * "USE_SharpZipLib"
 * 
 * Those methods where originally part of the framework but since it's rarely
 * used I've extracted this part into this seperate module file.
 * 
 * You can use the define "SimpleJSON_ExcludeBinary" to selectively disable
 * this extension without the need to remove the file from the project.
 * 
 * If you want to use compression when saving to file / stream / B64 you have to include
 * SharpZipLib ( http://www.icsharpcode.net/opensource/sharpziplib/ ) in your project and
 * define "USE_SharpZipLib" at the top of the file
 * 
 * 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2012-2017 Markus Göbel (Bunny83)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * * * * */
using System;

namespace SimpleJSON
{
#if !SimpleJSON_ExcludeBinary
    public abstract partial class JSONNode
    {
        public abstract void SerializeBinary(System.IO.BinaryWriter aWriter);

        public void SaveToBinaryStream(System.IO.Stream aData)
        {
            var W = new System.IO.BinaryWriter(aData);
            SerializeBinary(W);
        }

#if USE_SharpZipLib
		public void SaveToCompressedStream(System.IO.Stream aData)
		{
			using (var gzipOut = new ICSharpCode.SharpZipLib.BZip2.BZip2OutputStream(aData))
			{
				gzipOut.IsStreamOwner = false;
				SaveToBinaryStream(gzipOut);
				gzipOut.Close();
			}
		}
 
		public void SaveToCompressedFile(string aFileName)
		{
 
			System.IO.Directory.CreateDirectory((new System.IO.FileInfo(aFileName)).Directory.FullName);
			using(var F = System.IO.File.OpenWrite(aFileName))
			{
				SaveToCompressedStream(F);
			}
		}
		public string SaveToCompressedBase64()
		{
			using (var stream = new System.IO.MemoryStream())
			{
				SaveToCompressedStream(stream);
				stream.Position = 0;
				return System.Convert.ToBase64String(stream.ToArray());
			}
		}
 
#else
        public void SaveToCompressedStream(System.IO.Stream aData)
        {
            throw new Exception("Can't use compressed functions. You need include the SharpZipLib and uncomment the define at the top of SimpleJSON");
        }

        public void SaveToCompressedFile(string aFileName)
        {
            throw new Exception("Can't use compressed functions. You need include the SharpZipLib and uncomment the define at the top of SimpleJSON");
        }

        public string SaveToCompressedBase64()
        {
            throw new Exception("Can't use compressed functions. You need include the SharpZipLib and uncomment the define at the top of SimpleJSON");
        }
#endif

        public void SaveToBinaryFile(string aFileName)
        {
            System.IO.Directory.CreateDirectory((new System.IO.FileInfo(aFileName)).Directory.FullName);
            using (var F = System.IO.File.OpenWrite(aFileName))
            {
                SaveToBinaryStream(F);
            }
        }

        public string SaveToBinaryBase64()
        {
            using (var stream = new System.IO.MemoryStream())
            {
                SaveToBinaryStream(stream);
                stream.Position = 0;
                return System.Convert.ToBase64String(stream.ToArray());
            }
        }

        public static JSONNode DeserializeBinary(System.IO.BinaryReader aReader)
        {
            JSONNodeType type = (JSONNodeType)aReader.ReadByte();
            switch (type)
            {
                case JSONNodeType.Array:
                    {
                        int count = aReader.ReadInt32();
                        JSONArray tmp = new JSONArray();
                        for (int i = 0; i < count; i++)
                            tmp.Add(DeserializeBinary(aReader));
                        return tmp;
                    }
                case JSONNodeType.Object:
                    {
                        int count = aReader.ReadInt32();
                        JSONObject tmp = new JSONObject();
                        for (int i = 0; i < count; i++)
                        {
                            string key = aReader.ReadString();
                            var val = DeserializeBinary(aReader);
                            tmp.Add(key, val);
                        }
                        return tmp;
                    }
                case JSONNodeType.String:
                    {
                        return new JSONString(aReader.ReadString());
                    }
                case JSONNodeType.Number:
                    {
                        return new JSONNumber(aReader.ReadDouble());
                    }
                case JSONNodeType.Boolean:
                    {
                        return new JSONBool(aReader.ReadBoolean());
                    }
                case JSONNodeType.NullValue:
                    {
                        return JSONNull.CreateOrGet();
                    }
                default:
                    {
                        throw new Exception("Error deserializing JSON. Unknown tag: " + type);
                    }
            }
        }

#if USE_SharpZipLib
		public static JSONNode LoadFromCompressedStream(System.IO.Stream aData)
		{
			var zin = new ICSharpCode.SharpZipLib.BZip2.BZip2InputStream(aData);
			return LoadFromBinaryStream(zin);
		}
		public static JSONNode LoadFromCompressedFile(string aFileName)
		{
			using(var F = System.IO.File.OpenRead(aFileName))
			{
				return LoadFromCompressedStream(F);
			}
		}
		public static JSONNode LoadFromCompressedBase64(string aBase64)
		{
			var tmp = System.Convert.FromBase64String(aBase64);
			var stream = new System.IO.MemoryStream(tmp);
			stream.Position = 0;
			return LoadFromCompressedStream(stream);
		}
#else
        public static JSONNode LoadFromCompressedFile(string aFileName)
        {
            throw new Exception("Can't use compressed functions. You need include the SharpZipLib and uncomment the define at the top of SimpleJSON");
        }

        public static JSONNode LoadFromCompressedStream(System.IO.Stream aData)
        {
            throw new Exception("Can't use compressed functions. You need include the SharpZipLib and uncomment the define at the top of SimpleJSON");
        }

        public static JSONNode LoadFromCompressedBase64(string aBase64)
        {
            throw new Exception("Can't use compressed functions. You need include the SharpZipLib and uncomment the define at the top of SimpleJSON");
        }
#endif

        public static JSONNode LoadFromBinaryStream(System.IO.Stream aData)
        {
            using (var R = new System.IO.BinaryReader(aData))
            {
                return DeserializeBinary(R);
            }
        }

        public static JSONNode LoadFromBinaryFile(string aFileName)
        {
            using (var F = System.IO.File.OpenRead(aFileName))
            {
                return LoadFromBinaryStream(F);
            }
        }

        public static JSONNode LoadFromBinaryBase64(string aBase64)
        {
            var tmp = System.Convert.FromBase64String(aBase64);
            var stream = new System.IO.MemoryStream(tmp);
            stream.Position = 0;
            return LoadFromBinaryStream(stream);
        }
    }

    public partial class JSONArray : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {
            aWriter.Write((byte)JSONNodeType.Array);
            aWriter.Write(m_List.Count);
            for (int i = 0; i < m_List.Count; i++)
            {
                m_List[i].SerializeBinary(aWriter);
            }
        }
    }

    public partial class JSONObject : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {
            aWriter.Write((byte)JSONNodeType.Object);
            aWriter.Write(m_Dict.Count);
            foreach (string K in m_Dict.Keys)
            {
                aWriter.Write(K);
                m_Dict[K].SerializeBinary(aWriter);
            }
        }
    }

    public partial class JSONString : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {
            aWriter.Write((byte)JSONNodeType.String);
            aWriter.Write(m_Data);
        }
    }

    public partial class JSONNumber : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {
            aWriter.Write((byte)JSONNodeType.Number);
            aWriter.Write(m_Data);
        }
    }

    public partial class JSONBool : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {
            aWriter.Write((byte)JSONNodeType.Boolean);
            aWriter.Write(m_Data);
        }
    }
    public partial class JSONNull : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {
            aWriter.Write((byte)JSONNodeType.NullValue);
        }
    }
    internal partial class JSONLazyCreator : JSONNode
    {
        public override void SerializeBinary(System.IO.BinaryWriter aWriter)
        {

        }
    }
#endif
}