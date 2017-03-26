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

        MatrixTransform zoomTransform = new MatrixTransform();
        MatrixTransform scaleTransform = new MatrixTransform();

        internal void ZoomTo(double x, double width)
        {
            // figure out where this is given existing transform.
            Matrix mp = zoomTransform.Matrix;
            mp.OffsetX -= x;
            mp.Scale(this.ActualWidth / width, 1);
            zoomTransform.Matrix = mp;

            var info = ComputeScaleSelf(0);
            ApplyScale(info);

            InvalidateArrange();
        }

        internal void ResetZoom()
        {
            zoomTransform = new MatrixTransform();
            InvalidateArrange();
        }
        
        public void SetData(DataSeries series)
        {
            this.dirty = true;
            this.series = series;

            this.UpdateLayout();
            if (this.ActualWidth != 0)
            {
                UpdateChart();
            }
        }

        bool dirty;
        int scaleIndex; // for incremental scale calculation.

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
                ChartScaleInfo info = ComputeScaleSelf(this.scaleIndex);
                changed = ApplyScale(info);
            }
            return changed;
        }

        public ChartScaleInfo ComputeScaleSelf(int index)
        {
            if (scaleSelf == null)
            {
                scaleSelf = new ChartScaleInfo();
            }

            if (!dirty)
            {
                return scaleSelf;
            }

            if (index > 0)
            {
                // this is an incremental update then so we pick up where we left off.
            }
            else
            {
                // start over
                scaleSelf = new ChartScaleInfo();
            }

            if (series == null)
            {
                return scaleSelf;
            }

            int len = series.Values.Count;
            if (index < 0) index = 0;
            
            for (int i = index; i < len; i++)
            {
                DataValue d = series.Values[i];
                double x = d.X;
                double y = d.Y;
                scaleSelf.Add(x, y);
            }

            scaleIndex = len;

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

        int updateIndex;
        int startIndex;

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
                updateIndex = 0;
                redo = true;
            }

            if (ComputeScale() || redo)
            {
                UpdateChart();
                g = Graph.Data as PathGeometry;
            }
            else
            {
                AddScaledValues(f, updateIndex, series.Values.Count);
            }

            double dx = g.Bounds.Width - this.ActualWidth;
            if (dx > 0)
            {
                Canvas.SetLeft(Graph, -g.Bounds.Left - dx);
                UpdatePointer(lastMousePosition);
            }
            updateIndex = series.Values.Count;
        }


        void UpdateChart()
        {
            Canvas.SetLeft(Graph, 0);
            scaleIndex = 0;
            updateIndex = 0;

            ComputeScale();

            if (series.Values == null || series.Values.Count == 0)
            {
                Graph.Data = null;
                MinLabel.Text = "";
                MaxLabel.Text = "";
                AddTrendLineMenuItem.IsEnabled = false;
                AddMeanLineMenuItem.IsEnabled = false;
                AddSlidingVarianceMenuItem.IsEnabled = false;
                return;
            }
            if (liveScrolling && series.Values.Count > this.ActualWidth)
            {
                // just show the tail that fits on screen, since the scaling will not happen on x-axis in this case.
                updateIndex = series.Values.Count - (int)this.ActualWidth;
                scaleIndex = updateIndex;
                startIndex = updateIndex;
                minY = double.MaxValue;
                maxY = double.MinValue;
                minX = double.MaxValue;
                maxX = double.MinValue;
            }

            double count = series.Values.Count;
            PathGeometry g = new PathGeometry();
            PathFigure f = new PathFigure();
            g.Figures.Add(f);

            AddScaledValues(f, updateIndex, series.Values.Count);
            updateIndex = series.Values.Count;

            Graph.Data = g;
            Graph.Stroke = this.Stroke;
            Graph.StrokeThickness = this.StrokeThickness;

            UpdatePointer(lastMousePosition);
            AddTrendLineMenuItem.IsEnabled = true;
            AddMeanLineMenuItem.IsEnabled = true;
            AddSlidingVarianceMenuItem.IsEnabled = true;
        }

        private void AddScaledValues(PathFigure figure, int start, int end)
        {
            double height = this.ActualHeight - 1;
            double availableHeight = height;
            double width = this.ActualWidth;

            int len = series.Values.Count;
            double offset = Canvas.GetLeft(Graph);


            bool started = (figure.Segments.Count > 0);
            for (int i = start; i < end; i++)
            {
                DataValue d = series.Values[i];

                // add graph segment
                Point point = scaleTransform.Transform(new Point(d.X, d.Y));
                point = zoomTransform.Transform(point);
                double y = availableHeight - point.Y;
                double x = point.X;

                double rx = x + offset;
                if (rx > 0) 
                {
                    Point pt = new Point(x, y);
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


        const double TooltipThreshold = 20;

        DataValue FindNearestValue(Point pos, bool ignoreY)
        {
            DataValue found = null;

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

                    Point scaled = scaleTransform.Transform(new Point(d.X, d.Y));
                    scaled = zoomTransform.Transform(scaled);
                    if (ignoreY)
                    {
                        double x = scaled.X;
                        double distance = Math.Abs(x - pos.X);
                        if (distance < TooltipThreshold)
                        {
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                found = d;
                            }
                        }
                    }
                    else
                    {
                        double x = scaled.X;
                        double y = availableHeight - scaled.Y;
                        double dx = (x - pos.X);
                        double dy = y - pos.Y;
                        double distance = Math.Sqrt((dx * dx) + (dy * dy));
                        if (distance < TooltipThreshold)
                        {
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                found = d;
                            }
                        }
                    }
                }
            }
            return found;
        }

        void UpdatePointer(Point pos)
        {
            DataValue found = FindNearestValue(pos, false);
            if (found != null)
            {
                double availableHeight = this.ActualHeight;
                double value = found.Y;
                Point scaled = scaleTransform.Transform(new Point(found.X, found.Y));
                scaled = zoomTransform.Transform(scaled);
                double offset = Canvas.GetLeft(Graph);
                double x = scaled.X + offset;
                double y = availableHeight - scaled.Y;
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
                DataValue startData = FindNearestValue(localStartPos, true);
                DataValue endData = FindNearestValue(localEndPos, true);
                if (startData != null && endData != null)
                {
                    Point tipPosition = this.TransformToDescendant(Graph).Transform(localEndPos);
                    double microseconds = endData.X - startData.X;
                    TimeSpan span = new TimeSpan((long)microseconds * 10);
                    ShowTip(span.ToString(), tipPosition);
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

            double offset = Canvas.GetLeft(Graph);
            double tipPositionX = pos.X;// + offset;
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
        }

        void RepositionTip(PointerBorder pointer)
        {
            DataValue data = pointer.Data;

            double availableHeight = this.ActualHeight;
            double value = data.Y;
            Point scaled = scaleTransform.Transform(new Point(data.X, data.Y));
            scaled = zoomTransform.Transform(scaled);
            double offset = Canvas.GetLeft(Graph);
            double x = scaled.X + offset;
            double y = availableHeight - scaled.Y;
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
            Path ptr = new Path() {
                Fill = Pointer.Fill, Data = Pointer.Data.Clone(), RenderTransform = Pointer.RenderTransform.Clone() };
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

        private void OnAddTrendLine(object sender, RoutedEventArgs e)
        {
            if (this.series.Values.Count == 0)
            {
                return;
            }
            List<Point> points = new List<Point>(from d in this.series.Values select new Point(d.X, d.Y));
            double a, b; //  y = a + b.x
            MathHelpers.LinearRegression(points, out a, out b);

            DataValue first = this.series.Values.First();
            DataValue last = this.series.Values.Last();

            // now scale this line to fit the scaled graph
            Point start = new Point(first.X, a + (b * first.X));
            Point end = new Point(last.X, a + (b * last.X));

            double height = this.ActualHeight - 1;
            double availableHeight = height;
            double offset = Canvas.GetLeft(Graph);
            
            // scale start point
            Point point1 = scaleTransform.Transform(start);
            point1 = zoomTransform.Transform(point1);
            double y1 = availableHeight - point1.Y;

            // scale end point
            Point point2 = scaleTransform.Transform(end);
            point2 = zoomTransform.Transform(point2);
            double y2 = availableHeight - point2.Y;

            Line line = new Line() {
                Stroke = Graph.Stroke, StrokeThickness = 1, X1 = point1.X, Y1 = y1, X2 = point2.X, Y2 = y2,            
                StrokeDashArray = new DoubleCollection(new double[] { 2, 2 })
            };
            AdornerCanvas.Children.Add(line);

            TextBlock startLabel = new TextBlock() {
                Text = String.Format("{0:N3}", start.Y),
                Foreground = Graph.Stroke,
                Margin = new Thickness(point1.X, y1 + 2, 0, 0)
            };
            AdornerCanvas.Children.Add(startLabel);

            TextBlock endlabel = new TextBlock()
            {
                Text = String.Format("{0:N3}", end.Y),
                Foreground = Graph.Stroke
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
                const int WindowSize = 10;
                int count = 0;
                DataValue[] window = new DataValue[WindowSize];
                foreach (DataValue d in this.series.Values)
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
            List<double> points = new List<double>(from d in this.series.Values select d.Y);
            double mean = MathHelpers.Mean(points);

            DataValue first = this.series.Values.First();
            DataValue last = this.series.Values.Last();

            // now scale this line to fit the scaled graph
            Point start = new Point(first.X, mean);
            Point end = new Point(last.X, mean);

            double height = this.ActualHeight - 1;
            double availableHeight = height;
            double offset = Canvas.GetLeft(Graph);

            // scale start point
            Point point1 = scaleTransform.Transform(start);
            point1 = zoomTransform.Transform(point1);
            double y1 = availableHeight - point1.Y;

            // scale end point
            Point point2 = scaleTransform.Transform(end);
            point2 = zoomTransform.Transform(point2);
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
    }

    class PointerBorder : Border
    {
        public DataValue Data { get; set; }
        public Path Pointer { get; set; }
    }
}
