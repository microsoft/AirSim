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
    /// Interaction logic for ConnectorControl.xaml
    /// </summary>
    public partial class ConnectorControl : UserControl
    {
        public ConnectorControl()
        {
            InitializeComponent();
            OnConnectedChanged();
        }

        protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
        {
            base.OnRenderSizeChanged(sizeInfo);
            Size s = sizeInfo.NewSize;

            // the natural size is 72x32, so scale content to fit new size.
            // Width="72" Height="32"
            double xscale = s.Width / 72;
            double yscale = s.Height / 32;
            double scale = Math.Min(xscale, yscale);
            LayoutRoot.LayoutTransform = new ScaleTransform(scale, scale);
        }

        public bool Connected
        {
            get { return (bool)GetValue(ConnectedProperty); }
            set { SetValue(ConnectedProperty, value); }
        }

        // Using a DependencyProperty as the backing store for Connected.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty ConnectedProperty =
            DependencyProperty.Register("Connected", typeof(bool), typeof(ConnectorControl), new PropertyMetadata(false, new PropertyChangedCallback(OnConnectedChanged)));

        private static void OnConnectedChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((ConnectorControl)d).OnConnectedChanged();
        }

        private void OnConnectedChanged()
        {
            PathOpen.Visibility = Connected ? Visibility.Collapsed : Visibility.Visible;
            PathClosed.Visibility = Connected ? Visibility.Visible : Visibility.Collapsed;
        }
    }
}
