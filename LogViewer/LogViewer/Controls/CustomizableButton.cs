using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;

namespace LogViewer.Controls
{
    /// <summary>
    /// This class provides some additional properties on Button that is handy for themeing custom button ControlTemplates.
    /// </summary>
    public class CustomizableButton : Button
    {

        public CornerRadius CornerRadius
        {
            get { return (CornerRadius)GetValue(CornerRadiusProperty); }
            set { SetValue(CornerRadiusProperty, value); }
        }

        // Using a DependencyProperty as the backing store for CornerRadius.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty CornerRadiusProperty =
            DependencyProperty.Register("CornerRadius", typeof(CornerRadius), typeof(CustomizableButton), new PropertyMetadata(new CornerRadius(0)));

        public Brush MouseOverBackground
        {
            get { return (Brush)GetValue(MouseOverBackgroundProperty); }
            set { SetValue(MouseOverBackgroundProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MouseOverBackground.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MouseOverBackgroundProperty =
            DependencyProperty.Register("MouseOverBackground", typeof(Brush), typeof(CustomizableButton), new PropertyMetadata(null));


        public Brush MouseOverForeground
        {
            get { return (Brush)GetValue(MouseOverForegroundProperty); }
            set { SetValue(MouseOverForegroundProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MouseOverForeground.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MouseOverForegroundProperty =
            DependencyProperty.Register("MouseOverForeground", typeof(Brush), typeof(CustomizableButton), new PropertyMetadata(null));


        public Brush MouseOverBorder
        {
            get { return (Brush)GetValue(MouseOverBorderProperty); }
            set { SetValue(MouseOverBorderProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MouseOverBorder.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MouseOverBorderProperty =
            DependencyProperty.Register("MouseOverBorder", typeof(Brush), typeof(CustomizableButton), new PropertyMetadata(null));



        public Brush MousePressedBackground
        {
            get { return (Brush)GetValue(MousePressedBackgroundProperty); }
            set { SetValue(MousePressedBackgroundProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MousePressedBackground.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MousePressedBackgroundProperty =
            DependencyProperty.Register("MousePressedBackground", typeof(Brush), typeof(CustomizableButton), new PropertyMetadata(null));


        public Brush MousePressedBorder
        {
            get { return (Brush)GetValue(MousePressedBorderProperty); }
            set { SetValue(MousePressedBorderProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MousePressedBorder.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MousePressedBorderProperty =
            DependencyProperty.Register("MousePressedBorder", typeof(Brush), typeof(CustomizableButton), new PropertyMetadata(null));


        public Brush MousePressedForeground
        {
            get { return (Brush)GetValue(MousePressedForegroundProperty); }
            set { SetValue(MousePressedForegroundProperty, value); }
        }

        // Using a DependencyProperty as the backing store for MousePressedForeground.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty MousePressedForegroundProperty =
            DependencyProperty.Register("MousePressedForeground", typeof(Brush), typeof(CustomizableButton), new PropertyMetadata(null));

    }
}
