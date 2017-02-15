using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Input;

namespace LogViewer.Controls
{
    /// <summary>
    /// Captures and eats MouseWheel events so that a nested ListBox does not
    /// prevent an outer scrollable control from scrolling.
    /// </summary>
    public sealed class PassthroughMouseWheelBehavior
    {

        public static bool GetPassthroughMouseWheel(UIElement e)
        {
            return (bool)e.GetValue(PassthroughMouseWheelProperty);
        }

        public static void SetPassthroughMouseWheel(UIElement e, bool value)
        {
            e.SetValue(PassthroughMouseWheelProperty, value);
        }

        public static readonly DependencyProperty PassthroughMouseWheelProperty =
            DependencyProperty.RegisterAttached("PassthroughMouseWhee", typeof(bool),
            typeof(PassthroughMouseWheelBehavior), new UIPropertyMetadata(false, OnPassthroughMouseWheelChanged));

        static void OnPassthroughMouseWheelChanged(DependencyObject depObj, DependencyPropertyChangedEventArgs e)
        {
            var item = depObj as UIElement;
            if (item == null)
                return;

            if (e.NewValue is bool == false)
                return;

            if ((bool)e.NewValue)
            {
                item.PreviewMouseWheel += OnPreviewMouseWheel;
            }
            else
            {
                item.PreviewMouseWheel -= OnPreviewMouseWheel;
            }
        }

        static void OnPreviewMouseWheel(object sender, MouseWheelEventArgs e)
        {
            e.Handled = true;

            var e2 = new MouseWheelEventArgs(e.MouseDevice, e.Timestamp, e.Delta)
            { RoutedEvent = UIElement.MouseWheelEvent };

            var gv = sender as UIElement;
            if (gv != null) gv.RaiseEvent(e2);
        }

    }
}
