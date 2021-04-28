using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;

namespace LogViewer.Utilities
{
    class ProgressUtility
    {
        ProgressBar bar;
        DispatcherTimer progressTimer;
        double min;
        double max;
        double current;

        public ProgressUtility(ProgressBar bar)
        {
            this.bar = bar;
        }

        void StartProgressTimer()
        {
            if (progressTimer == null)
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    progressTimer = new DispatcherTimer();
                    progressTimer.Tick += OnProgressTick;
                    progressTimer.Interval = TimeSpan.FromMilliseconds(30);
                    progressTimer.Start();
                });
            }
        }

        void StopProgressTimer()
        {
            DispatcherTimer temp = progressTimer;
            progressTimer = null;

            if (temp != null)
            {
                UiDispatcher.RunOnUIThread(() =>
                {
                    temp.Stop();
                    temp.Tick -= OnProgressTick;
                });
            }
        }

        public void ShowProgress(double min, double max, double current)
        {
            this.min = min;
            this.max = max;
            this.current = current;

            if (this.min != this.max)
            {
                if (this.progressTimer == null)
                {
                    StartProgressTimer();
                }
            }
            else
            {
                if (this.progressTimer != null)
                {
                    StopProgressTimer();
                }
            }
        }

        private void OnProgressTick(object sender, object e)
        {
            if (min == max || max == current)
            {
                this.bar.Visibility = Visibility.Collapsed;
            }
            else
            {
                this.bar.Visibility = Visibility.Visible;
                this.bar.Minimum = min;
                this.bar.Maximum = max;
                this.bar.Value = current;
            }
        }

    }
}
