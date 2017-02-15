using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace LogViewer.Utilities
{
    class Raw8UBitmapDecoder 
    {
        List<BitmapFrame> frames;

        public ReadOnlyCollection<BitmapFrame> Frames { get { return new ReadOnlyCollection<BitmapFrame>(frames); } }

        public static Raw8UBitmapDecoder Create(byte[] pixels, int width, int height)
        {

            BitmapSource frame = BitmapSource.Create(width, height, 96, 96, PixelFormats.Gray8, null, pixels, width);
            List<BitmapFrame> frames = new List<BitmapFrame>();
            frames.Add(BitmapFrame.Create(frame));
            return new Raw8UBitmapDecoder() { frames = frames };
        }
        
    }
}
