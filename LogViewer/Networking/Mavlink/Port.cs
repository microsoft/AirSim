using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Microsoft.Networking.Mavlink
{
    public interface IPort
    {
        // write to the port
        void Write(byte[] buffer, int count);

        void Write(string msg);

        // read a given number of bytes from the port.
        int Read(byte[] buffer, int bytesToRead);

	    // close the port.
	    void Close();
    }
}
