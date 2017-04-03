using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace LogViewer.Controls
{
    /// <summary>
    /// Represents a vertical stack of charts or chartgroups
    /// </summary>
    public partial class ChartStack : Grid
    {
        public ChartStack()
        {
            InitializeComponent();
        }
        private bool selecting;
        private Point mouseDownPos;
        private int mouseDownTime;

        internal void AddChartGroup(ChartGroup chartGroup)
        {
            theStack.Children.Add(chartGroup);
        }

        public void AddChart(SimpleLineChart chart)
        {
            theStack.Children.Add(chart);
        }

        public void RemoveChart(SimpleLineChart e)
        {
            if (e.Parent == theStack)
            {
                theStack.Children.Remove(e);
            }
            else if (e.Parent is ChartGroup)
            {
                // it's a chart group then.
                ChartGroup group = (ChartGroup)e.Parent;
                group.Children.Remove(e);
                SimpleLineChart chart = e as SimpleLineChart;
                if (chart != null)
                {
                    chart.Group = null;
                }
                if (group.Children.Count == 0)
                {
                    theStack.Children.Remove(group);
                }
                group.InvalidateCharts();
            }
        }

        public void ClearCharts()
        {            
            foreach (var chart in FindCharts())
            {
                chart.Close();
                RemoveChart(chart);
            }
        }

        public int ChartCount {  get { return theStack.Children.Count; } }

        public UIElementCollection Charts {  get { return theStack.Children; } }

        protected override void OnKeyDown(KeyEventArgs e)
        {
            if (e.Key == Key.Escape)
            {
                selecting = false;
                Selection.Visibility = Visibility.Collapsed;
                e.Handled = true;
            }
        }

        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            selecting = true;
            Point pos = mouseDownPos = e.GetPosition(this);
            mouseDownTime = Environment.TickCount;
            Selection.Visibility = Visibility.Visible;
            Selection.Width = 1;
            Selection.Height = this.ActualHeight;
            Selection.Margin = new Thickness(pos.X, 0, 0, 0);
            Focus();
            this.CaptureMouse();
            base.OnMouseLeftButtonDown(e);
        }

        protected override void OnMouseLeftButtonUp(MouseButtonEventArgs e)
        {
            if (selecting)
            {
                int now = Environment.TickCount;
                int timeDelta = now - mouseDownTime;
                                    
                Point pos = e.GetPosition(this);

                double x = Math.Min(pos.X, mouseDownPos.X);
                double width = Math.Abs(pos.X - mouseDownPos.X);

                if (timeDelta > 100 && width > 10)
                {
                    ZoomTo(x, width);
                }

                selecting = false;
                Selection.Visibility = Visibility.Collapsed;

            }
            this.ReleaseMouseCapture();
            base.OnMouseLeftButtonUp(e);
        }

        List<UIElement> Snapshot(UIElementCollection collection)
        {
            List<UIElement> list = new List<UIElement>();
            foreach (UIElement e in collection)
            {
                list.Add(e);
            }
            return list;
        }

        public IEnumerable<SimpleLineChart> FindCharts()
        {
            foreach (UIElement e in Snapshot(theStack.Children))
            {
                SimpleLineChart chart = e as SimpleLineChart;
                if (chart != null)
                {
                    yield return chart;
                }
                ChartGroup group = e as ChartGroup;
                if (group != null)
                {
                    foreach (SimpleLineChart child in group.FindCharts())
                    {
                        yield return child;
                    }
                }
            }
        }

        private void ZoomTo(double x, double width)
        {
            foreach (SimpleLineChart chart in FindCharts())
            {
                chart.ZoomTo(x, width);
            }
            if (ZoomChanged != null)
            {
                ZoomChanged(this, EventArgs.Empty);
            }
        }

        public event EventHandler ZoomChanged;

        protected override void OnMouseMove(MouseEventArgs e)
        {
            Point pos = e.GetPosition(this);
            if (selecting)
            {
                double x = Math.Min(pos.X, mouseDownPos.X);
                Selection.Width = Math.Abs(pos.X - mouseDownPos.X);
                Selection.Margin = new Thickness(x, 0, 0, 0);

                foreach (SimpleLineChart chart in FindCharts())
                {
                    chart.HandleZoomTooltip(this, mouseDownPos, pos);
                }
            }
            else
            {
                foreach (SimpleLineChart chart in FindCharts())
                {
                    chart.HandleMouseMove(e);
                }
                e.Handled = false;
            }
            base.OnMouseMove(e);
        }

        protected override void OnMouseLeave(MouseEventArgs e)
        {
            foreach (SimpleLineChart chart in FindCharts())
            {
                chart.HandleMouseLeave();
            }
            base.OnMouseLeave(e);
        }

        protected override void OnLostMouseCapture(MouseEventArgs e)
        {
            selecting = false;
            Selection.Visibility = Visibility.Collapsed;
            base.OnLostMouseCapture(e);
        }

        internal void ResetZoom()
        {
            foreach (SimpleLineChart chart in FindCharts())
            {
                chart.ResetZoom();
            }
        }

    }
}
