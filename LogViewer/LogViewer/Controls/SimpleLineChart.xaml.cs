using LogViewer.Utilities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using LogViewer.Model;
using System.Diagnostics;
// The User Control item template is documented at http://go.microsoft.com/fwlink/?LinkId=234236

namespace LogViewer.Controls
{
    public class ChartScaleInfo
    {
        public double minX;
        public double maxX;
        public double minY;
        public double maxY;

        public ChartScaleInfo()
        {
            minY = double.MaxValue;
            maxY = double.MinValue;
            minX = double.MaxValue;
            maxX = double.MinValue;
        }

        public void Combine(ChartScaleInfo info)
        {
            minX = Math.Min(minX, info.minX);
            maxX = Math.Max(maxX, info.maxX);
            minY = Math.Min(minY, info.minY);
            maxY = Math.Max(maxY, info.maxY);
        }

        internal void Add(double x, double y)
        {
            minX = Math.Min(minX, x);
            maxX = Math.Max(maxX, x);
            minY = Math.Min(minY, y);
            maxY = Math.Max(maxY, y);
        }
    }

    public static class SimpleLineChartCommands
    {
        public readonly static RoutedUICommand CommandLockTooltip = new RoutedUICommand("Lock Tooltip", "LockTooltip", typeof(SimpleLineChart));
    }

    public sealed partial class SimpleLineChart : UserControl
    {
        private DataSeries series;
        private DispatcherTimer _updateTimer;
        private DelayedActions _delayedUpdates = new DelayedActions();
        DataValue currentValue;
        bool liveScrolling;
        double liveScrollingXScale = 1;
        static bool anyContextMenuOpen;
        double visibleCount;

        /// <summary>
        /// Set this property to add the chart to a group of charts.  The group will share the same "scale" information across the 
        /// combined chart group so that the charts line up under each other if they are arranged in a stack (if the group has
        /// ScaleIndependently set to true).
        /// </summary>
        public ChartGroup Group { get; set; }

        public SimpleLineChart()
        {
            this.InitializeComponent();
            this.Background = new SolidColorBrush(Colors.Transparent); // ensure we get manipulation events no matter where user presses.
            this.SizeChanged += SimpleLineChart_SizeChanged;

            this.ContextMenuOpening += ContextMenu_ContextMenuOpening;
            this.ContextMenuClosing += ContextMenu_ContextMenuClosing;

            EnableMenuItems();
        }

        internal double GetVisibleCount()
        {
            return visibleCount;
        }

        private void ContextMenu_ContextMenuOpening(object sender, ContextMenuEventArgs e)
        {
            anyContextMenuOpen = true;
        }

        private void ContextMenu_ContextMenuClosing(object sender, ContextMenuEventArgs e)
        {
            // tooltips are too quick to restart and lock tooltip picks up the new location before it can lock the original.
            _delayedUpdates.StartDelayedAction("SlowRestoreTooltips", () =>
            {
                anyContextMenuOpen = false;
            }, TimeSpan.FromSeconds(1));
        }

        void SimpleLineChart_SizeChanged(object sender, SizeChangedEventArgs e)
        {
            UpdateChart();
        }

        /// <summary>
        /// Call this method when you turn on LiveScrolling.
        /// </summary>
        public void SetCurrentValue(DataValue value)
        {
            System.Threading.Interlocked.Exchange<DataValue>(ref currentValue, value);
        }

        internal void Close()
        {
            if (Closed != null)
            {
                Closed(this, EventArgs.Empty);
            }
        }

        /// <summary>
        /// Set this to true to start the graph scrolling showing live values provided by thread safe
        /// SetCurrentValue method.
        /// </summary>
        public bool LiveScrolling
        {
            get { return liveScrolling; }
            set
            {
                liveScrolling = value;
                if (value)
                {
                    StartUpdateTimer();
                }
                else
                {
                    StopUpdateTimer();
                }
            }
        }

        /// <summary>
        /// For live scrolling we need to know how to scale the X values, this determines how fast the graph scrolls.
        /// </summary>
        public double LiveScrollingXScale
        {
            get { return liveScrollingXScale; }
            set
            {
                liveScrollingXScale = value;
                DelayedUpdate();
            }
        }

        double? fixedMinimumX;
        double? fixedMaximumX;

        /// <summary>
        /// If you do not want automatic calculation of min/max X scale, you can set these properties.
        /// </summary>
        public double? FixMinimumX
        {
            get { return fixedMinimumX; }
            set { fixedMinimumX = value; }
        }
        public double? FixMaximumX
        {
            get { return fixedMaximumX; }
            set { fixedMaximumX = value; }
        }


        private void StartUpdateTimer()
        {
            if (_updateTimer == null)
            {
                _updateTimer = new DispatcherTimer();
                _updateTimer.Interval = TimeSpan.FromMilliseconds(30);
                _updateTimer.Tick += OnUpdateTimerTick;
            }
            _updateTimer.Start();
        }


        private void StopUpdateTimer()
        {
            if (_updateTimer != null)
            {
                _updateTimer.Tick -= OnUpdateTimerTick;
                _updateTimer.Stop();
                _updateTimer = null;
            }
        }        

        void OnUpdateTimerTick(object sender, object e)
        {
            if (liveScrolling && currentValue != null)
            {
                SmoothScroll(currentValue);
            }
            else
            {
                UpdateChart();
            }
        }

        public void DelayedUpdate()
        {
            _delayedUpdates.StartDelayedAction("Update", UpdateChart, TimeSpan.FromMilliseconds(30));
        }
        
        MatrixTransform scaleTransform = new MatrixTransform();

        internal void ZoomTo(double x, double width)
        {
            int i = GetIndexFromX(x);
            if (i == -1)
            {
                // hmmmm?
                return;
            }
            this.visibleStartIndex = i;
            this.visibleEndIndex = GetIndexFromX(x + width);
            
            var info = ComputeScaleSelf();
            ApplyScale(info);

            InvalidateArrange();
        }

        internal void ResetZoom()
        {
            this.visibleStartIndex = 0;
            this.visibleEndIndex = -1;
            this.smoothScrollScaleIndex = 0;
            this.scaleSelf = new ChartScaleInfo();
            InvalidateArrange();
        }
        
        public void SetData(DataSeries series)
        {
            this.dirty = true;
            this.series = series;
            this.currentValue = null;
            this.scaleTransform = null;

            this.UpdateLayout();
            if (this.ActualWidth != 0)
            {
                UpdateChart();
            }
        }

        bool dirty;
        int visibleStartIndex;
        int visibleEndIndex = -1;
        int smoothScrollIndex;
        int smoothScrollScaleIndex;
        double xScale;
        double yScale;
        double minY;
        double maxY;
        double minX;
        double maxX;
        ChartScaleInfo scaleSelf;

        /// <summary>
        /// Returns true if the scale just changed.
        /// </summary>
        bool ComputeScale()
        {
            bool changed = false;

            if (this.Group != null && !scaleIndependently)
            {
                changed = this.Group.ComputeScale(this);
            }
            else
            {
                ChartScaleInfo info = ComputeScaleSelf();
                changed = ApplyScale(info);
            }
            return changed;
        }


        public ChartScaleInfo ComputeScaleSelf()
        {
            int startIndex = this.visibleStartIndex;
            int endIndex = this.visibleEndIndex;

            if (!dirty && scaleSelf != null)
            {
                return scaleSelf;
            }

            if (scaleSelf != null && this.liveScrolling)
            {
                // try and do incremental update of the scale for added efficiency!
                startIndex = this.smoothScrollScaleIndex;
            }
            else
            {
                scaleSelf = new ChartScaleInfo();
                this.smoothScrollScaleIndex = 0;
            }
            if (series == null)
            {
                return scaleSelf;
            }

            int len = series.Values.Count;
            if (startIndex < 0) startIndex = 0;
            if (endIndex< 0 || endIndex > len) endIndex = len;

            if (this.ActualHeight == 0)
            {
                return scaleSelf;
            }

            for (int i = startIndex; i < endIndex; i++)
            {
                DataValue d = series.Values[i];
                double x = d.X;
                double y = d.Y;
                scaleSelf.Add(x, y);
            }
            this.smoothScrollScaleIndex = endIndex;
            return scaleSelf;
        }

        internal bool ApplyScale(ChartScaleInfo info)
        {
            if (info == null)
            {
                return false;
            }

            double actualHeight = this.ActualHeight - 1;
            double actualWidth = this.ActualWidth;

            bool changed = false;
            this.minX = info.minX;
            this.maxX = info.maxX;
            this.minY = info.minY;
            this.maxY = info.maxY;

            double yRange = maxY - minY;
            if (yRange == 0)
            {
                yRange = 1;
            }
            double newyScale = actualHeight / yRange;
            if (newyScale == 0)
            {
                newyScale = 1;
            }
            if (newyScale != yScale)
            {
                yScale = newyScale;
                changed = true;
            }

            // overrides for automatic X-Scaling.
            if (fixedMaximumX.HasValue)
            {
                this.maxX = fixedMaximumX.Value;
            }

            if (fixedMinimumX.HasValue)
            {
                this.minX = fixedMinimumX.Value;
            }

            double newxScale = LiveScrollingXScale;
            if (!LiveScrolling)
            {
                double xRange = maxX - minX;
                if (xRange == 0) xRange = 1;
                newxScale = actualWidth / xRange;
                if (newxScale == 0)
                {
                    newxScale = 1;
                }
            }

            if (newxScale != xScale)
            {
                xScale = newxScale;
                changed = true;
            }

            if (changed || scaleTransform == null)
            {
                Matrix m = new Matrix();
                m.Scale(xScale, yScale);
                m.OffsetX = -minX * xScale;
                m.OffsetY = -minY * yScale;
                scaleTransform = new MatrixTransform(m);
            }

            return changed;
        }

        private void SmoothScroll(DataValue newValue)
        {
            if (this.series == null)
            {
                this.series = new DataSeries() { Name = "", Values = new List<DataValue>() };
            }
            
            DataValue copy = new Model.DataValue()
            {
                X = newValue.X,
                Y = newValue.Y
            };

            this.series.Values.Add(copy);

            PathGeometry g = Graph.Data as PathGeometry;
            if (g == null)
            {
                UpdateChart();
                return;
            }
            PathFigure f = g.Figures[0];

            bool redo = false;
            if (g.Bounds.Width > 2 * this.ActualWidth && this.series != null && this.series.Values.Count > 2 * this.ActualWidth)
            {
                // purge history since this is an infinite scrolling stream...
                this.series.Values.RemoveRange(0, this.series.Values.Count - (int)this.ActualWidth);
                System.Diagnostics.Debug.WriteLine("Trimming data series {0} back to {1} values", this.series.Name, this.series.Values.Count);
                dirty = true;
                redo = true;
            }

            if (ComputeScale() || redo)
            {
                UpdateChart();
                g = Graph.Data as PathGeometry;
            }
            else
            {
                this.visibleEndIndex = series.Values.Count;
                AddScaledValues(f, this.smoothScrollIndex, this.visibleEndIndex);
                this.smoothScrollIndex = this.series.Values.Count;
            }

            double dx = g.Bounds.Width - this.ActualWidth;
            if (dx > 0)
            {
                Canvas.SetLeft(Graph, -g.Bounds.Left - dx);
                UpdatePointer(lastMousePosition);
            }
        }

        void UpdateChart()
        {
            if (this.series == null)
            {
                this.series = new DataSeries() { Name = "", Values = new List<DataValue>() };
            }
            Canvas.SetLeft(Graph, 0);
            visibleCount = 0;
            this.smoothScrollIndex = 0;
            this.smoothScrollScaleIndex = 0;

            ComputeScale();

            if (series.Values == null || series.Values.Count == 0)
            {
                Graph.Data = null;
                MinLabel.Text = "";
                MaxLabel.Text = "";
                EnableMenuItems();
                return;
            }

            if (liveScrolling)
            {
                // just show the tail that fits on screen, since the scaling will not happen on x-axis in this case.
                var width = this.ActualWidth;
                
                this.visibleEndIndex = series.Values.Count;
                this.visibleStartIndex = this.visibleEndIndex;

                if (series.Values.Count > 0)
                {
                    // walk back until the scaled values fill one screen width.
                    this.visibleStartIndex = this.visibleEndIndex - 1;
                    Point endPoint = GetScaledValue(series.Values[this.visibleStartIndex]);
                    while (--this.visibleStartIndex > 0)
                    {
                        Point p = GetScaledValue(series.Values[this.visibleStartIndex]);
                        if (endPoint.X - p.X > width)
                        {
                            break;
                        }
                    }
                }

                minY = double.MaxValue;
                maxY = double.MinValue;
                minX = double.MaxValue;
                maxX = double.MinValue;
                this.smoothScrollIndex = this.series.Values.Count;

                ComputeScale();
            }

            double count = series.Values.Count;
            PathGeometry g = new PathGeometry();
            PathFigure f = new PathFigure();
            g.Figures.Add(f);

            AddScaledValues(f, this.visibleStartIndex, this.visibleEndIndex);

            Graph.Data = g;
            Graph.Stroke = this.Stroke;
            Graph.StrokeThickness = this.StrokeThickness;

            UpdatePointer(lastMousePosition);
            EnableMenuItems();
        }

        private void EnableMenuItems()
        {
            bool enabled = this.series != null && this.series.Values.Count > 0;
            AddTrendLineMenuItem.IsEnabled = enabled;
            AddMeanLineMenuItem.IsEnabled = enabled;
            ShowStatsMenuItem.IsEnabled = enabled;
            AddSlidingVarianceMenuItem.IsEnabled = enabled;
        }

        private void AddScaledValues(PathFigure figure, int start, int end)
        {
            double width = this.ActualWidth;            
            double offset = Canvas.GetLeft(Graph);
            if (end < 0 || end > this.series.Values.Count) end = this.series.Values.Count;
            if (start < 0) start = 0;

            bool started = (figure.Segments.Count > 0);
            for (int i = start; i < end; i++)
            {
                DataValue d = series.Values[i];

                // add graph segment
                Point pt = GetScaledValue(d);

                double rx = pt.X + offset;

                if (pt.X >= 0)
                {
                    visibleCount++;
                    if (!started)
                    {
                        figure.StartPoint = pt;
                        started = true;
                    }
                    else
                    {
                        figure.Segments.Add(new LineSegment() { Point = pt });
                    }
                }
            }
        }

        public Brush Stroke
        {
            get { return (Brush)GetValue(StrokeProperty); }
            set { SetValue(StrokeProperty, value); }
        }

        // Using a DependencyProperty as the backing store for Stroke.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty StrokeProperty =
            DependencyProperty.Register("Stroke", typeof(Brush), typeof(SimpleLineChart), new PropertyMetadata(new SolidColorBrush(Colors.White)));



        public double StrokeThickness
        {
            get { return (double)GetValue(StrokeThicknessProperty); }
            set { SetValue(StrokeThicknessProperty, value); }
        }

        // Using a DependencyProperty as the backing store for StrokeThickness.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty StrokeThicknessProperty =
            DependencyProperty.Register("StrokeThickness", typeof(double), typeof(SimpleLineChart), new PropertyMetadata(1.0));

        private void CloseBoxClick(object sender, RoutedEventArgs e)
        {
            Close();
        }

        public event EventHandler Closed;
        public Point lastMousePosition;

        internal void HandleMouseMove(MouseEventArgs e)
        {
            if (!anyContextMenuOpen)
            {
                lastMousePosition = e.GetPosition(this);
                UpdatePointer(lastMousePosition);
            }
        }

        internal void HandleMouseLeave()
        {
            if (!anyContextMenuOpen)
            {
                HidePointer();
            }
        }
        
        protected override void OnMouseMove(MouseEventArgs e)
        {
            if (anyContextMenuOpen)
            {
                // don't move tooltip while user is interacting with the menu.
                return;
            }
            HandleMouseMove(e);
            base.OnMouseMove(e);
        }

        protected override void OnMouseLeave(MouseEventArgs e)
        {
            HandleMouseLeave();
            base.OnMouseLeave(e);
        }

        public Color LineColor
        {
            get
            {
                var brush = this.Stroke as SolidColorBrush;
                if (brush != null)
                {
                    return brush.Color;
                }

                return Colors.Black;
            }
            set
            {
                this.Stroke = new SolidColorBrush(value);

                HlsColor darker = new HlsColor(value);
                darker.Darken(0.33f);
                PointerBorder.BorderBrush = Pointer.Fill = PointerLabel.Foreground = new SolidColorBrush(darker.Color);

            }
        }

        int GetIndexFromX(double x)
        {
            // transform top Graph coordinates (which could be constantly changing because of zoom and scrolling.
            Point pos = this.TransformToDescendant(Graph).Transform(new Point(x, 0));
            x = pos.X;

            if (series.Values != null && series.Values.Count > 0)
            {
                for (int i = 0; i < series.Values.Count; i++)
                {
                    DataValue d = series.Values[i];
                    Point scaled = GetScaledValue(d);
                    if (scaled.X >= x)
                    {
                        return i;
                    }
                }
            }
            return -1;
        }

        const double TooltipThreshold = 20;

        int FindNearestValue(Point pos, bool ignoreY)
        {
            int found = -1;

            // transform top Graph coordinates (which could be constantly changing because of zoom and scrolling.
            pos = this.TransformToDescendant(Graph).Transform(pos);

            if (series.Values != null && series.Values.Count > 0)
            {
                // add graph segment
                double availableHeight = this.ActualHeight;
                double width = this.ActualWidth;

                double minDistance = double.MaxValue;

                // find the closed data value.

                for (int i = 0; i < series.Values.Count; i++)
                {
                    DataValue d = series.Values[i];

                    Point scaled = GetScaledValue(d);
                    if (ignoreY)
                    {
                        double x = scaled.X;
                        double distance = Math.Abs(x - pos.X);
                        if (distance < TooltipThreshold)
                        {
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                found = i;
                            }
                        }
                    }
                    else
                    {
                        double x = scaled.X;
                        double y = scaled.Y;
                        double dx = (x - pos.X);
                        double dy = y - pos.Y;
                        double distance = Math.Sqrt((dx * dx) + (dy * dy));
                        if (distance < TooltipThreshold)
                        {
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                found = i;
                            }
                        }
                    }
                }
            }
            return found;
        }

        void UpdatePointer(Point pos)
        {
            int i = FindNearestValue(pos, false);
            if (i >= 0)
            {
                DataValue found = series.Values[i];
                Point pt = GetScaledValue(found);
                double offset = Canvas.GetLeft(Graph);
                double x = pt.X + offset;
                double y = pt.Y;
                ShowTip(series.Name + " = " + (!string.IsNullOrEmpty(found.Label) ? found.Label : found.Y.ToString()), new Point(x, y), found);
            }
            else
            {
                HidePointer();
            }
        }

        internal void HandleZoomTooltip(ChartStack stack, Point mouseDownPos, Point pos)
        {
            Point localStartPos = stack.TransformToDescendant(this).Transform(mouseDownPos);
            Point localEndPos = stack.TransformToDescendant(this).Transform(pos);
            if (localEndPos.X >= 0 && localEndPos.Y >= 0 && localEndPos.X < this.ActualWidth && localEndPos.Y < this.ActualHeight)
            {
                int s = FindNearestValue(localStartPos, true);
                int e = FindNearestValue(localEndPos, true);
                if (s >= 0 && e >= 0)
                {
                    DataValue startData = series.Values[s];
                    DataValue endData = series.Values[e];
                    Point tipPosition = this.TransformToDescendant(Graph).Transform(localEndPos);
                    double microseconds = endData.X - startData.X;
                    TimeSpan span = new TimeSpan((long)microseconds * 10);
                    double seconds = span.TotalSeconds;
                    double diff = endData.Y - startData.Y;
                    double sum = 0;
                    double count = e - s;
                    for (int i = s; i <= e; i++)
                    {
                        DataValue d = series.Values[i];
                        sum += d.Y;
                    }
                    double average = sum / count;
                    string msg = string.Format("{0:N3} sec, avg={1:N4}, dist={2:N3}, rate={3:N3}, samples={4:N3}", seconds, average, diff, diff / seconds, e - s);
                    ShowTip(msg, tipPosition);
                }
                else
                {
                    HidePointer();
                }
            }
        }

        void HidePointer()
        {
            if (PointerBorder.Visibility == Visibility.Visible)
            {
                PointerBorder.Visibility = System.Windows.Visibility.Hidden;
            }
            Pointer.Visibility = System.Windows.Visibility.Hidden;
            LockTooltipMenuItem.IsEnabled = false;
        }

        void ShowTip(string label, Point pos, DataValue data = null)
        {
            PointerLabel.Text = label;
            PointerBorder.UpdateLayout();
            
            double tipPositionX = pos.X;
            if (tipPositionX + PointerBorder.ActualWidth > this.ActualWidth)
            {
                tipPositionX = this.ActualWidth - PointerBorder.ActualWidth;
            }
            double tipPositionY = pos.Y - PointerLabel.ActualHeight - 4;
            if (tipPositionY < 0)
            {
                tipPositionY = 0;
            }
            PointerBorder.Margin = new Thickness(tipPositionX, tipPositionY, 0, 0);
            PointerBorder.Visibility = System.Windows.Visibility.Visible;
            PointerBorder.Data = data;

            Point pointerPosition = pos;
            Pointer.RenderTransform = new TranslateTransform(pointerPosition.X, pointerPosition.Y);
            Pointer.Visibility = System.Windows.Visibility.Visible;

            LockTooltipMenuItem.IsEnabled = true;

            if (PointerMoved != null)
            {
                PointerMoved(this, data);
            }
        }

        public event EventHandler<DataValue> PointerMoved;

        void RepositionTip(PointerBorder pointer)
        {
            DataValue data = pointer.Data;
            Point pt = GetScaledValue(data);
            double offset = Canvas.GetLeft(Graph);
            double x = pt.X + offset;
            double y = pt.Y;
            double tipPositionX = x;
            if (tipPositionX + PointerBorder.ActualWidth > this.ActualWidth)
            {
                tipPositionX = this.ActualWidth - PointerBorder.ActualWidth;
            }
            double tipPositionY = y - PointerLabel.ActualHeight - 4;
            if (tipPositionY < 0)
            {
                tipPositionY = 0;
            }
            pointer.Margin = new Thickness(tipPositionX, tipPositionY, 0, 0);

            pointer.Pointer.RenderTransform = new TranslateTransform(x, y);
        }


        protected override Size ArrangeOverride(Size arrangeBounds)
        {
            this.dirty = true;
            DelayedUpdate();
            HidePointer();
            _delayedUpdates.StartDelayedAction("RepositionTips", () => { RepositionTips(); }, TimeSpan.FromMilliseconds(30));
            return base.ArrangeOverride(arrangeBounds);
        }

        private void RepositionTips()
        {
            foreach (UIElement child in AdornerCanvas.Children)
            {
                PointerBorder pointer = child as PointerBorder;
                if (pointer != null)
                {
                    RepositionTip(pointer);
                }
            }
        }

        private void OnLockTooltip(object sender, RoutedEventArgs e)
        {
            // we lock the tooltip by cloning the existing tooltip objects.
            /*
            <Path x:Name="Pointer" Fill="{StaticResource TooltipForeground}" Data="M0,-5 L 5,0 0,5 -5 0z" Visibility="Collapsed"/>
            <Border x:Name="PointerBorder"  Visibility="Collapsed" Padding="2" HorizontalAlignment="Left" VerticalAlignment="Top" BorderThickness="1"
                    CornerRadius="3" BorderBrush="{StaticResource TooltipForeground}" Background="#80303030">
                <TextBlock x:Name="PointerLabel" Foreground="{StaticResource TooltipForeground}"/>
            </Border>
             */
            Path ptr = new Path()
            {
                Fill = Pointer.Fill,
                Data = Pointer.Data.Clone(),
                RenderTransform = Pointer.RenderTransform.Clone()
            };
            AdornerCanvas.Children.Add(ptr);

            PointerBorder ptrBorder = new PointerBorder()
            {
                Padding = PointerBorder.Padding,
                HorizontalAlignment = PointerBorder.HorizontalAlignment,
                VerticalAlignment = PointerBorder.VerticalAlignment,
                BorderThickness = PointerBorder.BorderThickness,
                CornerRadius = PointerBorder.CornerRadius,
                BorderBrush = PointerBorder.BorderBrush,
                Background = PointerBorder.Background,
                Margin = PointerBorder.Margin,
                Data = PointerBorder.Data,
                Pointer = ptr,
            };
            ptrBorder.Child = new TextBlock() { Foreground = PointerLabel.Foreground, Text = PointerLabel.Text };
            AdornerCanvas.Children.Add(ptrBorder);
        }

        private IEnumerable<DataValue> GetVisibleDataValues()
        {
            double w = this.ActualWidth;
            double offset = Canvas.GetLeft(Graph);
            if (this.series != null)
            {
                foreach (DataValue d in this.series.Values)
                {
                    Point point = GetScaledValue(d);
                    if (point.X >= 0 && point.X <= w)
                    {
                        // then it is a visible point.                        
                        yield return d;
                    }
                }
            }
        }

        private Point GetScaledValue(DataValue d)
        {
            double availableHeight = this.ActualHeight - 1;
            Point point = scaleTransform.Transform(new Point(d.X, d.Y));
            return new Point(point.X, availableHeight - point.Y);
        }

        private IEnumerable<Point> GetVisibleScaledValues()
        {
            double offset = Canvas.GetLeft(Graph);
            double w = this.ActualWidth;
            if (this.series != null)
            {
                foreach (DataValue d in this.series.Values)
                {
                    Point point = GetScaledValue(d);
                    point.X += offset;
                    if (point.X >= 0 && point.X <= w)
                    {
                        // then it is a visible point.                        
                        yield return point;
                    }
                }
            }
        }

        private Tuple<int,int> GetVisibleRange()
        {
            double offset = Canvas.GetLeft(Graph);
            double w = this.ActualWidth;
            int start = 0;
            bool started = false;
            int end = 0;
            Point point;
            if (this.series != null)
            {
                for (int i = 0, length = this.series.Values.Count; i < length; i++)
                {
                    DataValue d = this.series.Values[i];
                    point = GetScaledValue(d);
                    point.X += offset;
                    if (point.X >= 0 && point.X <= w)
                    {
                        if (started)
                        {
                            end = i;
                        }
                        else
                        {
                            start = i;
                            started = true;
                        }
                    }
                }
            }
            return new Tuple<int, int>(start, end);
        }

        private void OnAddTrendLine(object sender, RoutedEventArgs e)
        {
            if (this.series.Values.Count == 0)
            {
                return;
            }
            // only do the visible points we have zoomed into.
            List<Point> points = new List<Point>();

            DataValue first = null;
            DataValue last = null;
            double w = this.ActualWidth;

            foreach (DataValue d in GetVisibleDataValues())
            {
                // then it is a visible point.
                if (first == null) first = d;
                last = d;
                points.Add(new Point(d.X, d.Y));
            }
            if (first == null)
            {
                return;
            }
            double a, b; //  y = a + b.x
            MathHelpers.LinearRegression(points, out a, out b);

            Point start = new Point(first.X, a + (b * first.X));
            Point end = new Point(last.X, a + (b * last.X));

            double height = this.ActualHeight - 1;
            double availableHeight = height;
            
            // scale start point
            Point point1 = scaleTransform.Transform(start);
            double y1 = availableHeight - point1.Y;

            // scale end point
            Point point2 = scaleTransform.Transform(end);
            double y2 = availableHeight - point2.Y;

            Line line = new Line() {
                Stroke = Graph.Stroke, StrokeThickness = 1, X1 = point1.X, Y1 = y1, X2 = point2.X, Y2 = y2,            
                StrokeDashArray = new DoubleCollection(new double[] { 2, 2 })
            };
            AdornerCanvas.Children.Add(line);

            TextBlock startLabel = new TextBlock() {
                Text = String.Format("{0:N3}", start.Y),
                Foreground = Brushes.White,
                Margin = new Thickness(point1.X, y1 + 2, 0, 0)
            };
            AdornerCanvas.Children.Add(startLabel);

            TextBlock endlabel = new TextBlock()
            {
                Text = String.Format("{0:N3}", end.Y),
                Foreground = Brushes.White
            };
            endlabel.SizeChanged += (s, args) =>
            {
                endlabel.Margin = new Thickness(point2.X - args.NewSize.Width - 10, y2 + 2, 0, 0);
            };

            AdornerCanvas.Children.Add(endlabel);
        }

        private void OnClearAdornments(object sender, RoutedEventArgs e)
        {
            AdornerCanvas.Children.Clear();
        }

        public void ClearAdornments()
        {
            AdornerCanvas.Children.Clear();
        }

        public event EventHandler ClearAllAdornments;

        private void OnClearAllAdornments(object sender, RoutedEventArgs e)
        {
            if (ClearAllAdornments != null)
            {
                ClearAllAdornments(this, EventArgs.Empty);
            }
        }

        private void OnScaleIndependently(object sender, RoutedEventArgs e)
        {
            bool value = !ScaleIndependentMenuItem.IsChecked;
            ScaleIndependentMenuItem.IsChecked = value;

            if (this.Group != null)
            {
                this.Group.ScaleIndependently = value;
            }
            else
            {
                scaleIndependently = value;
            }
        }

        bool scaleIndependently = false;

        public bool ScaleIndependently
        {
            get { return scaleIndependently;  }
            set
            {
                if (scaleIndependently != value)
                {
                    scaleIndependently = value;
                    DelayedUpdate();
                }
            }
        }

        public event EventHandler<List<DataValue>> ChartGenerated;

        private void OnAddSlidingVariance(object sender, RoutedEventArgs e)
        {
            if (ChartGenerated != null)
            {
                List<DataValue> result = new List<Model.DataValue>();
                const int WindowSize = 30;
                int count = 0;
                DataValue[] window = new DataValue[WindowSize];
                foreach (DataValue d in this.GetVisibleDataValues())
                {
                    window[count] = d;
                    count++;
                    if (count == WindowSize)
                    {
                        double variance = MathHelpers.Variance(from s in window select s.Y);
                        double time = window[0].X;
                        result.Add(new DataValue() { X = time, Y = variance });
                        count = 0; // restart the window.
                    }
                }
                ChartGenerated(this, result);
            }
        }

        private void OnAddMeanLine(object sender, RoutedEventArgs e)
        {
            if (this.series.Values.Count == 0)
            {
                return;
            }
            // only do the visible points we have zoomed into.
            List<double> points = new List<double>();

            DataValue first = null;
            DataValue last = null;

            foreach (DataValue d in GetVisibleDataValues())
            {
                // then it is a visible point.
                if (first == null) first = d;
                last = d;
                points.Add(d.Y);
            }
            if (first == null)
            {
                return;
            }
            double mean = MathHelpers.Mean(points);
            
            // now scale this line to fit the scaled graph
            Point start = new Point(first.X, mean);
            Point end = new Point(last.X, mean);

            double height = this.ActualHeight - 1;
            double availableHeight = height;

            // scale start point
            Point point1 = scaleTransform.Transform(start);
            double y1 = availableHeight - point1.Y;

            // scale end point
            Point point2 = scaleTransform.Transform(end);
            double y2 = availableHeight - point2.Y;

            Line line = new Line()
            {
                Stroke = Brushes.White,
                StrokeThickness = 1,
                X1 = point1.X,
                Y1 = y1,
                X2 = point2.X,
                Y2 = y2,
                StrokeDashArray = new DoubleCollection(new double[] { 2, 2 })
            };
            AdornerCanvas.Children.Add(line);

            TextBlock startLabel = new TextBlock()
            {
                Text = String.Format("{0:N3}", start.Y),
                Foreground = Brushes.White,
                Margin = new Thickness(point1.X, y1 + 2, 0, 0)
            };
            AdornerCanvas.Children.Add(startLabel);

            TextBlock endlabel = new TextBlock()
            {
                Text = String.Format("{0:N3}", end.Y),
                Foreground = Brushes.White
            };
            endlabel.SizeChanged += (s, args) =>
            {
                endlabel.Margin = new Thickness(point2.X - args.NewSize.Width - 10, y2 + 2, 0, 0);
            };

            AdornerCanvas.Children.Add(endlabel);
        }

        private void OnExportCsv(object sender, RoutedEventArgs e)
        {
            Microsoft.Win32.SaveFileDialog fo = new Microsoft.Win32.SaveFileDialog();
            fo.Filter = "CSV Files (*.csv)|*.csv";
            fo.CheckPathExists = true;
            if (fo.ShowDialog() == true)
            {
                using (var stream = fo.OpenFile())
                {
                    using (System.IO.StreamWriter writer = new System.IO.StreamWriter(stream))
                    {
                        if (this.series != null)
                        {
                            writer.WriteLine("ticks\t" + this.series.Name);
                            foreach (var d in this.GetVisibleDataValues())
                            {
                                writer.WriteLine(d.X + "\t" + d.Y);
                            }
                       }
                    }
                }
            }
        }

        public event EventHandler<string> DisplayMessage;

        private void OnShowStats(object sender, RoutedEventArgs e)
        {
            // only do the visible points we have zoomed into.
            List<double> points = new List<double>();
            double min = double.MaxValue;
            double max = double.MinValue;
            int count = 0;
            foreach (DataValue d in GetVisibleDataValues())
            {
                // then it is a visible point.
                points.Add(d.Y);
                min = Math.Min(min, d.Y);
                max = Math.Max(max, d.Y);
                count++;
            }
            double mean = MathHelpers.Mean(points);
            double variance = MathHelpers.Variance(points);
            double stddev = MathHelpers.StandardDeviation(points);

            if (DisplayMessage != null)
            {
                DisplayMessage(this, string.Format(@"Analyzing {0} data values from '{6}':
  minimum  {1} ({1:0.####E+0}), 
  maximum  {2} ({2:0.####E+0})
  mean     {3} ({3:0.####E+0}), 
  variance {4} ({4:0.####E+0}),
  stddev   {5} ({5:0.####E+0})",
  count, min, max, mean, variance, stddev, this.series.Name));
            }
        }
    }

    class PointerBorder : Border
    {
        public DataValue Data { get; set; }
        public Path Pointer { get; set; }
    }
}
