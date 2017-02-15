using LogViewer.Utilities;
using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;

namespace LogViewer
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        Settings settings;

        public async Task<Settings> LoadSettings()
        {
            if (this.settings == null) {
                this.settings = await Settings.LoadAsync();
            }
            return this.settings;
        }
        
    }
}
