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
    /// Interaction logic for SelectionOverlay.xaml
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

        internal void AddChartGroup(Grid chartGroup)
        {
            theStack.Children.Add(chartGroup);
        }

        public void AddChart(SimpleLineChart chart)
        {
            theStack.Children.Add(chart);
        }

        internal void AddToGroup(Grid chartGroup, FrameworkElement chart)
        {
            chartGroup.Children.Add(chart);

            if (chartGroup.Parent == null)
            {
                this.AddChartGroup(chartGroup);
            }

            FixNextPointers(chartGroup);
        }


        public void RemoveChart(SimpleLineChart e)
        {
            if (e.Parent == theStack)
            {
                theStack.Children.Remove(e);
            }
            else if (e.Parent is Grid)
            {
                // it's a chart group then.
                Grid group = (Grid)e.Parent;
                group.Children.Remove(e);
                SimpleLineChart chart = e as SimpleLineChart;
                if (chart != null)
                {
                    chart.Next = null;
                }
                if (group.Children.Count == 0)
                {
                    theStack.Children.Remove(group);
                }
                else
                {
                    FixNextPointers(group);
                }
            }
        }

        private static void FixNextPointers(Grid group)
        {
            // fix up the "next" pointers for removed chart.
            SimpleLineChart first = null;
            SimpleLineChart previous = null;
            foreach (UIElement f in group.Children)
            {
                SimpleLineChart chart = f as SimpleLineChart;
                if (chart != null)
                {
                    if (first == null)
                    {
                        first = chart;
                        chart.Next = null;
                    }
                    else
                    {
                        previous.Next = chart;
                        chart.Next = first;
                    }
                    previous = chart;
                }
            }
            foreach (UIElement f in group.Children)
            {
                SimpleLineChart chart = f as SimpleLineChart;
                if (chart != null)
                {
                    chart.InvalidateArrange();
                }
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


        protected override void OnMouseLeftButtonDown(MouseButtonEventArgs e)
        {
            selecting = true;
            Point pos = mouseDownPos = e.GetPosition(this);
            mouseDownTime = Environment.TickCount;
            Selection.Visibility = Visibility.Visible;
            Selection.Width = 1;
            Selection.Height = this.ActualHeight;
            Selection.Margin = new Thickness(pos.X, 0, 0, 0);
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
                Grid group = e as Grid;
                if (group != null)
                {
                    foreach (UIElement f in Snapshot(group.Children))
                    {
                        chart = f as SimpleLineChart;
                        if (chart != null)
                        {
                            yield return chart;
                        }
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
        }

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
