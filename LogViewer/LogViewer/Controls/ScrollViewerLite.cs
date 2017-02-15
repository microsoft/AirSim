using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;
using LogViewer.Utilities;
using System.Windows;
using System.Windows.Input;

namespace LogViewer.Controls
{
    class ScrollViewerLite : ScrollViewer
    {
        Border scrollerBorder = null;

        protected override void OnScrollChanged(ScrollChangedEventArgs e)
        {
            ShowScroller();
            base.OnScrollChanged(e);
        }

        private void ShowScroller()
        {
            if (scrollerBorder == null)
            {
                scrollerBorder = (from border in this.FindDescendantsOfType<Border>()
                                  where border.Name == "PART_ScrollBorder"
                                  select border).FirstOrDefault();
            }
            if (scrollerBorder != null)
            {
                bool rc = VisualStateManager.GoToElementState(scrollerBorder, "Scrolling", true);
                delayedActions.StartDelayedAction("HideScroller", () =>
                {
                    HideScroller();
                }, TimeSpan.FromSeconds(2));
            }
        }

        private void HideScroller()
        {
            if (Mouse.Captured != null)
            {
                // try again after mouse capture is cleared (user might be dragging the scrollbar thumb!)
                delayedActions.StartDelayedAction("HideScroller", () =>
                {
                    HideScroller();
                }, TimeSpan.FromSeconds(2));
            }
            else
            {
                VisualStateManager.GoToElementState(scrollerBorder, "NotScrolling", true);
            }
        }

        DelayedActions delayedActions = new DelayedActions();

    }
}
