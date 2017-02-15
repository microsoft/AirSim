using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Media.Media3D;

namespace LogViewer.Gestures
{
    interface IMouseGesture
    {
        /// <summary>
        /// Gesture has moved the model.
        /// </summary>
        event EventHandler Changed;

        /// <summary>
        /// This is the rotation delta being requested by the mouse gesture at any given time.
        /// </summary>
        Quaternion Rotation { get; set; }

        /// <summary>
        /// This is the zoom level delta being applied by the gesture to modify camera position.
        /// </summary>
        double Zoom { get; set; }
    }
}
