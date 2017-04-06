using LogViewer.Utilities;
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
    /// Interaction logic for AppSettings.xaml
    /// </summary>
    public partial class AppSettings : UserControl
    {
        public AppSettings()
        {
            InitializeComponent();

            List<AppTheme> items = new List<AppTheme>();
            items.Add(AppTheme.Light);
            items.Add(AppTheme.Dark);
            ThemeSelection.ItemsSource = items;
            ThemeSelection.SelectedItem = Settings.Instance.Theme;
            ThemeSelection.SelectionChanged += ThemeSelection_SelectionChanged;
        }

        private void OnCloseClicked(object sender, RoutedEventArgs e)
        {
            this.Visibility = Visibility.Collapsed;
        }

        private void ThemeSelection_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            if (e.AddedItems != null && e.AddedItems.Count > 0)
            {
                Settings.Instance.Theme = (AppTheme)e.AddedItems[0];
            }
        }

    }
}
