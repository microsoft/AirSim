using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace LogViewer.Gestures
{
    static class Extensions3D
    {
        public static Vector3D ToVector3D(this Point3D pt)
        {
            return new Vector3D(pt.X, pt.Y, pt.Z);
        }
        public static Point3D ToPoint3D(this Vector3D pt)
        {
            return new Point3D(pt.X, pt.Y, pt.Z);
        }
    }
}
