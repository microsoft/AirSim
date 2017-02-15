using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LogViewer.Utilities
{
    /// <summary>
    /// This class provides a delayed action that has a name.  If the same named action is
    /// started multiple times before the delay if fires the action only once.
    /// </summary>
    public class DelayedActions
    {
        Dictionary<string, DelayedAction> pending = new Dictionary<string, DelayedAction>();

        public void StartDelayedAction(string name, Action action, TimeSpan delay)
        {
            DelayedAction da;
            if (!pending.TryGetValue(name, out da))
            {
                da = new DelayedAction();
                pending[name] = da;
            }
            da.StartDelayTimer(action, delay);
        }

        public void CancelDelayedAction(string name)
        {
            DelayedAction action;
            if (pending.TryGetValue(name, out action))
            {
                action.StopDelayTimer();
                pending.Remove(name);
            }
        }

        public void Close()
        {
            foreach (var pair in pending)
            {
                pair.Value.StopDelayTimer();
            }
            pending.Clear();
        }


        class DelayedAction
        {
            System.Threading.Timer delayTimer;
            Action delayedAction;
            int startTime;

            /// <summary>
            /// Start a count down with the given delay, and fire the given action when it reaches zero.
            /// But if this method is called again before the timeout it resets the timeout and starts again.
            /// </summary>
            /// <param name="action">The action to perform when the delay is reached</param>
            /// <param name="delay">The timeout before calling the action</param>
            public void StartDelayTimer(Action action, TimeSpan delay)
            {

                startTime = Environment.TickCount;

                // stop any previous timer and start over.
                StopDelayTimer();

                this.delayedAction = action;

                this.delayTimer = new System.Threading.Timer(OnDelayTimerTick, null, (int)delay.TotalMilliseconds, System.Threading.Timeout.Infinite);
            }

            public void StopDelayTimer()
            {
                System.Threading.Timer timer = this.delayTimer;
                System.Threading.Interlocked.CompareExchange(ref this.delayTimer, null, timer);
                if (timer != null)
                {
                    // give up on this old one and start over.
                    timer.Dispose();
                    timer = null;
                }
                delayedAction = null;
            }

            private void OnDelayTimerTick(object state)
            {
                int endTime = Environment.TickCount;
                int diff = startTime - endTime;

                Action a = this.delayedAction;

                StopDelayTimer();

                if (a != null)
                {
                    UiDispatcher.RunOnUIThread(() =>
                    {
                        try
                        {
                            a();
                        }
                        catch (Exception ex)
                        {
                            Debug.WriteLine("OnDelayTimerTick caught unhandled exception: " + ex.ToString());
                        }
                    });
                }
            }
        }
    }
}
