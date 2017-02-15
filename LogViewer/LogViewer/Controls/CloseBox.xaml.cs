using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace LogViewer.Controls
{
    /// <summary>
    /// Interaction logic for CloseBox.xaml
    /// </summary>
    public partial class CloseBox : CustomizableButton
    {

        Brush normalBackground;
        Brush normalBorder;
        Brush normalForeground;

        public CloseBox()
        {
            InitializeComponent();
        }

        void SaveNormalColors()
        {
            if (normalBackground == null)
            {
                normalBackground = this.Background;
                normalForeground = this.Foreground;
                normalBorder = this.BorderBrush;
            }
        }

        void UpdateColors()
        {
            SaveNormalColors();

            Brush background = normalBackground;
            Brush foreground = normalForeground;

            if (IsMouseOver)
            {
                background = this.MouseOverBackground;
                foreground = this.MouseOverForeground;
            }

            if (IsPressed)
            {
                background = this.MousePressedBackground;
                foreground = this.MousePressedForeground;
            }

            this.Foreground = foreground;
            this.Background = background;
        }

        protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
        {
            base.OnRenderSizeChanged(sizeInfo);
            
            Size s = sizeInfo.NewSize;
            Rect inner = new Rect(0, 0, s.Width, s.Height);
            double radius = s.Width - this.BorderThickness.Left;
            double sinX = radius / Math.Sqrt(2);
            double margin = (int)(s.Width - sinX);
            
            inner.Inflate(-margin, -margin);
            
            Path p = (Path)Template.FindName("CrossShape", this);
            p.Data = new PathGeometry(new PathFigure[] 
            {
                new PathFigure(new Point(inner.Left, inner.Top),
                    new PathSegment[] {
                        new LineSegment(new Point(inner.Right, inner.Bottom), true),
                        new LineSegment(new Point(inner.Left, inner.Bottom), false),
                        new LineSegment(new Point(inner.Right, inner.Top), true),
                    }, false)
            });
        }

        protected override void OnMouseEnter(System.Windows.Input.MouseEventArgs args)
        {
            base.OnMouseEnter(args);
            Dispatcher.BeginInvoke(new Action(() =>
            {
                UpdateColors();
            }));
        }

        protected override void OnMouseLeave(System.Windows.Input.MouseEventArgs e)
        {
            base.OnMouseLeave(e);
            Dispatcher.BeginInvoke(new Action(() =>
            {
                UpdateColors();
            }));
        }

        protected override void OnMouseDown(System.Windows.Input.MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);
            Dispatcher.BeginInvoke(new Action(() =>
            {
                UpdateColors();
            }));
        }

        protected override void OnMouseUp(System.Windows.Input.MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);
            Dispatcher.BeginInvoke(new Action(() =>
            {
                UpdateColors();
            }));
        }
    }

}
