// Copyright (c) 2010 Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft
// license agreement under which you licensed this source code.
// If you did not accept the terms of the license agreement,
// you are not authorized to use this source code.
// For the terms of the license, please see the license agreement
// signed by you and Microsoft.
// THE SOURCE CODE IS PROVIDED "AS IS", WITH NO WARRANTIES OR INDEMNITIES.
//
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Xml.Serialization;

namespace Microsoft.Storage
{

    /// <summary>
    /// Isolated storage file helper class
    /// </summary>
    /// <typeparam name="T">Data type to serialize/deserialize</typeparam>
    public class IsolatedStorage<T>
    {
        Dictionary<string, int> locks = new Dictionary<string, int>();        

        public IsolatedStorage()
        {
        }

        class FileLock : IDisposable
        {
            Dictionary<string, int> locks;
            string key ;

            public FileLock(Dictionary<string, int> locks, string path)
            {
                this.locks = locks;
                this.key = path;
            }

            public void Dispose()
            {
                lock (this.locks)
                {
                    this.locks.Remove(this.key);
                }
            }
        }

        private IDisposable EnterLock(string path)
        {
            lock (locks)
            {
                int value = 0;
                if (locks.TryGetValue(path, out value))
                {
                    throw new Exception(string.Format("Re-entrant access to file: {0}", path));
                }
                else
                {
                    locks[path] = 1;
                }
                return new FileLock(locks, path);
            }
        }

        /// <summary>
        /// Loads data from a file asynchronously.
        /// </summary>
        /// <param name="folder">The folder to get the file from</param>
        /// <param name="fileName">Name of the file to read.</param>
        /// <returns>Deserialized data object</returns>
        public async Task<T> LoadFromFileAsync(string folder, string fileName)
        {
            T loadedFile = default(T);

            string fullPath = Path.Combine(folder, fileName);
            using (var l = EnterLock(fullPath))
            {
                try
                {
                    if (fullPath != null)
                    {
                        Debug.WriteLine("Loading file: {0}", fullPath);
                        await Task.Run(() =>
                        {
                            using (Stream myFileStream = File.OpenRead(fullPath))
                            {
                                // Call the Deserialize method and cast to the object type.
                                loadedFile = LoadFromStream(myFileStream);
                            }
                        });
                    }
                }
                catch
                {
                    // silently rebuild data file if it got corrupted.
                }
            }
            return loadedFile;
        }

        public T LoadFromStream(Stream s)
        {
            // Call the Deserialize method and cast to the object type.
            XmlSerializer mySerializer = new XmlSerializer(typeof(T));
            return (T)mySerializer.Deserialize(s);
        }

        /// <summary>
        /// Saves data to a file.
        /// </summary>
        /// <param name="fileName">Name of the file to write to</param>
        /// <param name="data">The data to save</param>
        public async Task SaveToFileAsync(string folder, string fileName, T data)
        {
            string path = System.IO.Path.Combine(folder, fileName);
            using (var l = EnterLock(path))
            {
                try
                {
                    await Task.Run(() =>
                    {
                        using (var stream = File.Create(path))
                        {
                            XmlSerializer mySerializer = new XmlSerializer(typeof(T));
                            mySerializer.Serialize(stream, data);
                        }
                    });
                }
                catch (Exception ex)
                {
                    Debug.WriteLine("### SaveToFileAsync failed: {0}", ex.Message);
                }
            }
        }

    }
}
