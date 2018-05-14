using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Animation;
using System.Windows.Media.Imaging;

namespace LogViewer.Utilities
{
    public static class XamlExtensions
    {
        public static void Flyout(this FrameworkElement e)
        {
            e.Visibility = Visibility.Visible;
            e.UpdateLayout();
            double width = e.ActualWidth;
            TranslateTransform transform = new TranslateTransform(width, 0);
            e.RenderTransform = transform;
            transform.BeginAnimation(TranslateTransform.XProperty,
                new DoubleAnimation(0, new Duration(TimeSpan.FromSeconds(0.2)))
                {
                    EasingFunction = new ExponentialEase() { EasingMode = EasingMode.EaseOut }
                });
        }

        public static T FindAncestorOfType<T>(this DependencyObject d) where T : DependencyObject
        {
            while (!(d is T) && d != null)
            {
                d = VisualTreeHelper.GetParent(d);
            }
            return (T)d;
        }

        public static IEnumerable<T> FindDescendantsOfType<T>(this DependencyObject d) where T : DependencyObject
        {
            List<T> result = new List<T>();
            CollectDescendantsOfType(d, result);
            return result;
        }

        private static void CollectDescendantsOfType<T>(DependencyObject d, List<T> result) where T : DependencyObject
        {
            if (typeof(T).IsAssignableFrom(d.GetType()))
            {
                result.Add((T)d);
            }
            bool content = (d is ContentElement);

            if (content || d is FrameworkElement)
            {
                //use the logical tree for content / framework elements
                foreach (object obj in LogicalTreeHelper.GetChildren(d))
                {
                    var child = obj as DependencyObject;
                    if (child != null)
                    {
                        CollectDescendantsOfType(child, result);
                    }
                }
            }
            
            if (!content)
            {
                //use the visual tree per default
                int count = VisualTreeHelper.GetChildrenCount(d);
                for (int i = 0; i < count; i++)
                {
                    var child = VisualTreeHelper.GetChild(d, i); 
                    CollectDescendantsOfType(child, result);
                }
            }
        }

        public static BitmapFrame LoadImage(string filePath)
        {
            using (FileStream fs = new FileStream(filePath, FileMode.Open, FileAccess.Read))
            {
                return LoadImage(fs);
            }
        }

        public static BitmapFrame LoadImage(Stream stream)
        {
            // Load it into memory first so we stop the BitmapDecoder from locking the file.
            MemoryStream ms = new MemoryStream();
            byte[] buffer = new byte[16000];
            
            int len = stream.Read(buffer, 0, buffer.Length);
            while (len > 0)
            {
                ms.Write(buffer, 0, len);
                len = stream.Read(buffer, 0, buffer.Length);
            }
            ms.Seek(0, SeekOrigin.Begin);

            BitmapDecoder decoder = BitmapDecoder.Create(ms, BitmapCreateOptions.IgnoreImageCache, BitmapCacheOption.None);
            BitmapFrame frame = decoder.Frames[0];

            return frame;
        }

        public static BitmapFrame LoadImageResource(string resourceName)
        {
            using (Stream s = typeof(XamlExtensions).Assembly.GetManifestResourceStream("LogViewer." + resourceName))
            {
                return LoadImage(s);
            }
        }


        [DllImport("GDI32.dll")]

        private static extern int GetDeviceCaps(HandleRef hDC, int nIndex);

        [DllImport("User32.dll")]
        private static extern IntPtr GetDC(HandleRef hWnd);

        [DllImport("User32.dll")]
        private static extern int ReleaseDC(HandleRef hWnd, HandleRef hDC);

        private static int _dpi = 0;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA1806")]
        private static int DPI
        {
            get
            {
                if (_dpi == 0)
                {
                    HandleRef desktopHwnd = new HandleRef(null, IntPtr.Zero);
                    HandleRef desktopDC = new HandleRef(null, GetDC(desktopHwnd));
                    _dpi = GetDeviceCaps(desktopDC, 88 /*LOGPIXELSX*/);
                    ReleaseDC(desktopHwnd, desktopDC);
                }
                return _dpi;
            }
        }

        internal static double ConvertToDeviceIndependentPixels(int pixels)
        {
            return (double)pixels * 96 / (double)DPI;
        }


        internal static int ConvertFromDeviceIndependentPixels(double pixels)
        {
            return (int)(pixels * (double)DPI / 96);
        }


    }
}
