using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;

namespace LogViewer.Utilities
{
    public static class MathHelpers
    {
        /// <summary>
        /// Return the Mean of the given numbers.
        /// </summary>
        public static double Mean(IEnumerable<double> values)
        {
            double sum = 0;
            double count = 0;
            foreach (double d in values)
            {
                sum += d;
                count++;
            }
            if (count == 0) return 0;
            return sum / count;
        }

        public static double StandardDeviation(IEnumerable<double> values)
        {
            double mean = Mean(values);
            double totalSquares = 0;
            int count = 0;
            foreach (double v in values)
            {
                count++;
                double diff = mean - v;
                totalSquares += diff * diff;
            }
            if (count == 0) 
            {
                return 0;
            }
            return Math.Sqrt((double)(totalSquares / (double)count));
        }

        /// <summary>
        /// Return the variance, sum of the difference between each value and the mean, squared.
        /// </summary>
        public static double Variance(IEnumerable<double> values)
        {
            double mean = Mean(values);
            double variance = 0;
            double count = 0;
            foreach (double d in values)
            {
                double diff = (d - mean);
                variance += (diff * diff);
                count++;
            }
            return variance / count;
        }

        /// <summary>
        /// Return the covariance in the given x,y values.
        /// The sum of the difference between x and its mean times the difference between y and its mean.
        /// </summary>
        public static double Covariance(IEnumerable<Point> pts)
        {
            double xsum = 0;
            double ysum = 0;
            double count = 0;
            foreach (Point d in pts)
            {
                xsum += d.X;
                ysum += d.Y;
                count++;
            }
            if (count == 0) return 0;
            double xMean = xsum / count;
            double yMean = ysum / count;
            double covariance = 0;
            foreach (Point d in pts)
            {
                covariance += (d.X - xMean) * (d.Y - yMean);
            }
            return covariance;
        }

        /// <summary>
        /// Compute the trend line through the given points, and return the line in the form:
        ///     y = a + b.x
        /// </summary>
        /// <param name="pts">The data to analyze</param>
        /// <param name="a">The y-coordinate of the line at x = 0</param>
        /// <param name="b">The slope of the line</param>
        public static void LinearRegression(IEnumerable<Point> pts, out double a, out double b)
        {
            double xMean = Mean(from p in pts select p.X);
            double yMean = Mean(from p in pts select p.Y);
            double xVariance = Variance(from p in pts select p.X);
            double yVariance = Variance(from p in pts select p.Y);
            double covariance = Covariance(pts);
            if (xVariance == 0)
            {
                a = yMean;
                b = 1;
            }
            else
            {
                b = covariance / xVariance;
                a = yMean - (b * xMean);
            }
        }
    }
}
