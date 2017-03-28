using Microsoft.Storage;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;

namespace LogViewer.Utilities
{ 
    public enum AppTheme
    {
        Light,
        Dark
    }

    public class Settings : INotifyPropertyChanged
    {
        const string SettingsFileName = "settings.xml";
        string fileName;
        Point windowLocation;
        Size windowSize;
        int port = 14550;
        AppTheme theme = AppTheme.Dark;

        static Settings _instance;

        public Settings()
        {
            _instance = this;
        }

        public static string SettingsFolder
        {
            get
            {
                string appSetttingsPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData), @"Microsoft\LogViewer");
                Directory.CreateDirectory(appSetttingsPath);
                return appSetttingsPath;
            }
        }

        public static Settings Instance
        {
            get
            {
                if (_instance == null)
                {
                    return new Settings();
                }
                return _instance;
            }
        }

        public Point WindowLocation
        {
            get { return this.windowLocation; }
            set { this.windowLocation  = value; }
        }

        public Size WindowSize
        {
            get { return this.windowSize; }
            set { this.windowSize = value; }
        }

        public int Port
        {
            get { return this.port; }
            set { this.port = value; }
        }

        public AppTheme Theme
        {
            get { return this.theme; }
            set { this.theme = value; }
        }

        public string LastLogFile
        {
            get
            {
                return this.fileName;
            }
            set
            {
                if (this.fileName != value)
                {
                    this.fileName = value;
                    OnPropertyChanged("LastLogFile");
                }
            }
        }
    

        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged(string name)
        {
            if (PropertyChanged != null)
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    PropertyChanged(this, new PropertyChangedEventArgs(name));
                });
            }
        }

        public static async Task<Settings> LoadAsync()
        {
            var store = new IsolatedStorage<Settings>();
            Settings result = null;
            try
            {
                Debug.WriteLine("Loading settings from : " + SettingsFolder);
                result = await store.LoadFromFileAsync(SettingsFolder, SettingsFileName);
            }
            catch
            {
            }
            if (result == null)
            {
                result = new Settings();
                await result.SaveAsync();
            }
            return result;
        }

        bool saving;

        public async Task SaveAsync()
        {
            var store = new IsolatedStorage<Settings>();
            if (!saving)
            {
                saving = true;
                try
                {
                    Debug.WriteLine("Saving settings to : " + SettingsFolder);
                    await store.SaveToFileAsync(SettingsFolder, SettingsFileName, this);
                }
                finally
                {
                    saving = false;
                }
            }
        }

    }


}
