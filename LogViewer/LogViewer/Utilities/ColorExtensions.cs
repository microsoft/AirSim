using System;
using System.Windows.Media;

namespace LogViewer.Utilities
{
    /// <summary>
    /// Helper extensions for manipulating colors.
    /// </summary>
    public static class ColorExtensions
    {

        public static int GetBrightness(this Color color)
        {
            return ((color.R * 299) + (color.G * 587) + (color.B * 114)) / 1000;
        }

        public static int GetDifference(this Color color1, Color color2)
        {
            return Math.Abs(color1.R - color2.R) + Math.Abs(color1.G - color2.G) + Math.Abs(color1.B - color2.B);
        }

        public static bool HasGoodContrast(this Color color1, Color color2)
        {
            int b1 = color1.GetBrightness();
            int b2 = color2.GetBrightness();
            return (Math.Abs(b2 - b1) > 125 && color1.GetDifference(color2) > 400);
        }

        public static bool HasReasonableContrast(this Color color1, Color color2)
        {
            int b1 = color1.GetBrightness();
            int b2 = color2.GetBrightness();
            return (Math.Abs(b2 - b1) > 50 && color1.GetDifference(color2) > 100);
        }


        /// <summary>
        /// Augment or reduce the Byte (RGB) by half of the delta between the Source and the target
        /// </summary>
        /// <param name="colorToken">Byte to start from</param>
        /// <param name="colorTargetToken">Byte to tareget</param>
        /// <returns>new byte value</returns>
        private static byte DeltaColorToken(int colorToken, int colorTargetToken)
        {
            if (colorToken == colorTargetToken)
            {
                return (byte)colorToken;
            }


            if (colorToken > colorTargetToken)
            {
                int delta = colorToken - colorTargetToken;
                return (byte)Math.Max(0, colorToken - (delta / 2));
            }
            else
            {
                int delta = colorTargetToken - colorToken;
                return (byte)Math.Min(255, colorToken + (delta / 2));
            }
        }


        /// <summary>
        /// Bring a color closer to another color
        /// </summary>
        /// <param name="color1">color to move</param>
        /// <param name="targetColor">targeted color</param>
        /// <returns>new color</returns>
        public static Color Interpolate(this Color color1, Color targetColor)
        {
            Color toReturn = color1;
            toReturn.A = DeltaColorToken(toReturn.A, targetColor.A);
            toReturn.R = DeltaColorToken(toReturn.R, targetColor.R);
            toReturn.G = DeltaColorToken(toReturn.G, targetColor.G);
            toReturn.B = DeltaColorToken(toReturn.B, targetColor.B);
            return toReturn;
        }


        /// <summary>
        /// Interpolate the color that is at the given target offset 
        /// </summary>
        /// <param name="color1">The first color</param>
        /// <param name="offset1">The position of the first color as a number between 0 and 1</param>
        /// <param name="color2">The second color</param>
        /// <param name="offset2">The position of the second color as a number between 0 and 1</param>
        /// <param name="targetOffset">The target position we are trying to interpolate as a number between 0 and 1</param>
        /// <returns>The color at the target offset</returns>
        public static Color LinearInterpolate(this Color color1, double offset1, Color color2, double offset2, double targetOffset)
        {
            if (offset1 < 0 || offset1 > 1)
            {
                throw new ArgumentOutOfRangeException("offset1");
            }
            if (offset2 < 0 || offset2 > 1)
            {
                throw new ArgumentOutOfRangeException("offset1");
            }
            if (targetOffset < 0 || targetOffset > 1)
            {
                throw new ArgumentOutOfRangeException("offset1");
            }

            if (offset1 == offset2) return color1;

            return Color.FromArgb((byte)LinearInterpolation(offset1, color1.A, offset2, color2.A, targetOffset),
                (byte)LinearInterpolation(offset1, color1.R, offset2, color2.R, targetOffset),
                (byte)LinearInterpolation(offset1, color1.G, offset2, color2.G, targetOffset),
                (byte)LinearInterpolation(offset1, color1.B, offset2, color2.B, targetOffset));
        }

        /// <summary>
        /// Get the contrast color for the given foreground and background colors
        /// </summary>
        /// <param name="foreground">The foreground color</param>
        /// <param name="background">The background color</param>
        /// <param name="alpha">The alpha-channel value to adjust the opacity of the resulting color</param>
        /// <param name="lessStrict">Use less strict contrast setting</param>
        /// <returns></returns>
        public static Color GetContrastColor(this Color foreground, Color background, byte alpha, bool lessStrict = false)
        {
            Color result;

            if (!foreground.HasReasonableContrast(background) ||
                (!lessStrict && !foreground.HasGoodContrast(background)))
            {
                // pick white or black, whichever one has the greatest difference.
                if (Colors.White.GetDifference(background) > Colors.Black.GetDifference(background))
                {
                    result = Colors.White;
                }
                else
                {
                    result = Colors.Black;
                }
            }
            else
            {
                result = foreground;
            }

            // Adjust the alpha-channel (transparency) if necessary
            if (result.A != alpha)
                result.A = alpha;

            return result;

        }

        /// <summary>
        /// This method is similar to GetContrastColor, only it is less aggressive, it allows the
        /// colors to get a bit closer before resorting to switching colors.
        /// </summary>
        /// <returns></returns>
        public static Color GetReasonableContrastColor(this Color foreground, Color background)
        {
            // Now check the contrast.
            if (!foreground.HasReasonableContrast(background))
            {
                // pick white or black, whichever one has the greatest difference.
                if (Colors.White.GetDifference(background) > Colors.Black.GetDifference(background))
                {
                    return Colors.White;
                }
                else
                {
                    return Colors.Black;
                }
            }
            return foreground;
        }



        /// <summary>
        /// Interpolate the position on a line.
        /// </summary>
        /// <param name="x1">The first x coordinate</param>
        /// <param name="y1">The first y coordinate</param>
        /// <param name="x2">The second x coordinate</param>
        /// <param name="y2">The second y coordinate</param>
        /// <param name="x3">The third x coordinate</param>
        /// <returns>return the value y3</returns>
        private static double LinearInterpolation(double x1, double y1, double x2, double y2, double x3)
        {
            // horizontal line.
            if (y1 == y2) return y1;

            if (x2 == x1) return (y1 + y2) / 2;

            // y = a + b.x
            double b = (y2 - y1) / (x2 - x1);

            double a = y1 - (b * x1);

            return a + (b * x3);
        }

    }
}
