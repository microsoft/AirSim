using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using LogViewer.Model;

namespace LogViewer.Controls
{
    public class HierarchicalLogItemSchemaStyleSelector : StyleSelector
    {
        Style containerStyle;
        Style leafStyle;

        public override Style SelectStyle(object item, DependencyObject d)
        {
            FrameworkElement element = d as FrameworkElement;

            if (element != null && item != null && item is LogItemSchema)
            {
                LogItemSchema category = item as LogItemSchema;

                if (category.HasChildren)
                {
                    if (containerStyle == null)
                    {
                        containerStyle = element.FindResource("ContainerListItemStyle") as Style;
                    }
                    return containerStyle;
                }
                else
                {
                    if (leafStyle == null)
                    {
                        leafStyle = element.FindResource("ChildListItemStyle") as Style;
                    }
                    return leafStyle;
                }
            }

            return null;
        }
    }

    public class HierarchicalLogItemSchemaTemplateSelector : DataTemplateSelector
    {
        DataTemplate container;
        DataTemplate leaf;

        public override DataTemplate SelectTemplate(object item, DependencyObject d)
        {
            FrameworkElement element = d as FrameworkElement;

            if (element != null && item != null && item is LogItemSchema)
            {
                LogItemSchema category = item as LogItemSchema;

                if (category.HasChildren)
                {
                    if (container == null)
                    {
                        container = element.FindResource("ContainerLogItemSchemaTemplate") as DataTemplate;
                    }
                    return container;
                }
                else
                {
                    if (leaf == null)
                    {
                        leaf = element.FindResource("LeafLogItemSchemaTemplate") as DataTemplate;
                    }
                    return leaf;
                }
            }

            return null;
        }
    }
}
