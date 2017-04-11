using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;

namespace LogViewer.Controls
{
    /// <summary>
    ///  Represents a group of chats located on top of each other.
    /// </summary>
    public class ChartGroup : Grid
    {
        private bool scaleIndependently;

        public bool ScaleIndependently
        {
            get { return scaleIndependently; }
            set
            {
                if (scaleIndependently != value)
                {
                    scaleIndependently = value;
                    InvalidateCharts();
                }
            }
        }

        public void InvalidateCharts()
        {
            foreach (var chart in FindCharts())
            {
                chart.ScaleIndependently = this.scaleIndependently;
                chart.InvalidateArrange();
                chart.DelayedUpdate();
            }
        }


        internal void AddChart(SimpleLineChart chart)
        {
            this.Children.Add(chart);
            chart.Group = this;
            InvalidateCharts();
        }



        public IEnumerable<SimpleLineChart> FindCharts()
        {
            List<SimpleLineChart> result = new List<SimpleLineChart>();
            foreach (UIElement f in this.Children)
            {
                SimpleLineChart chart = f as SimpleLineChart;
                if (chart != null)
                {
                    result.Add(chart);
                }
            }
            return result;
        }
        
        internal bool ComputeScale(SimpleLineChart trigger)
        {
            bool changed = false;
            ChartScaleInfo combined = null;

            // make sure they are all up to date.
            foreach (var ptr in FindCharts())
            {
                ChartScaleInfo info = ptr.ComputeScaleSelf();
                if (combined == null)
                {
                    combined = info;
                }
                else
                {
                    combined.Combine(info);
                }
            }

            foreach (var ptr in FindCharts())
            {
                if (ptr.ApplyScale(combined))
                {
                    if (ptr != trigger)
                    {
                        ptr.DelayedUpdate();
                    }
                    changed = true;
                }
            }
            return changed;
        }
    }
}
