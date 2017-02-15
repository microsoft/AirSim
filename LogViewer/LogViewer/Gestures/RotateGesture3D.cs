using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

namespace LogViewer.Gestures
{
    class RotateGesture3D : IMouseGesture
    {
        bool m_tracking;
        Point m_start;
        FrameworkElement container;
        double m_XRotation;
        double m_YRotation;
        double m_PreviousXRotation;
        double m_degreesXVelocity;
        double m_degreesXAcceleration;
        double m_PreviousYRotation;
        double m_degreesYVelocity;
        double m_degreesYAcceleration;
        double m_Zoom;
        Quaternion m_StartRotation;
        DispatcherTimer flickTimer;
        double breakingAcceleration;

        public RotateGesture3D(FrameworkElement container)
        {
            this.container = container;
            container.MouseLeftButtonDown += OnMouseLeftButtonDown;
            container.MouseLeftButtonUp += OnMouseLeftButtonUp;
            container.MouseMove += OnMouseMove;
            container.MouseWheel += OnMouseWheel;
            container.LostMouseCapture += OnLostMouseCapture;
            Sensitivity = 100;
            BrakingAcceleration = 0.05;
            MinFlickThreshold = 5.0;
        }

        public event EventHandler Changed;        

        private void OnChanged()
        {
            // movement in the Y dimension is interpretted as rotation about the horizontal axis
            RotateTransform3D startTransform = new RotateTransform3D(new QuaternionRotation3D(m_StartRotation));
            Quaternion yRotation = new Quaternion(startTransform.Inverse.Transform(new Point3D(1, 0, 0)).ToVector3D(), this.m_YRotation);
            RotateTransform3D yTransform = new RotateTransform3D(new QuaternionRotation3D(yRotation));
            // movement in the X dimension is interpretted as rotation about the vertical axis.
            var xAxis = yTransform.Transform(new Point3D(0, 0, 1)).ToVector3D();
            Quaternion xRotation = new Quaternion(xAxis, this.m_XRotation);
            this.Rotation = m_StartRotation * (xRotation * yRotation);

            if (Changed != null)
            {
                Changed(this, EventArgs.Empty);
            }
        }

                
        /// <summary>
        /// This is the rotation being applied by the user.
        /// </summary>
        public Quaternion Rotation { get; set; }


        public double Zoom
        {
            get { return m_Zoom; }
            set { m_Zoom = value; }
        }


        private void OnMouseWheel(object sender, System.Windows.Input.MouseWheelEventArgs e)
        {
            int clicks = e.Delta;
            if (clicks > 0)
            {
                m_Zoom--;
            }
            else
            {
                m_Zoom++;
            }
            OnChanged();
        }

        const double XM_PI = 3.141592654f;
        const double XM_2PI = 6.283185307f;

        public double Sensitivity { get; set; }

        public double BrakingAcceleration
        {
            get { return breakingAcceleration; }
            set
            {
                breakingAcceleration = Math.Min(Math.Max(value,0),1);
            }
        }

        public double MinFlickThreshold { get; set; }

        private void OnMouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if (m_tracking)
            {
                // Rotation about the mouse down position.
                Point position = e.GetPosition(container);

                // XRotation is rotation about the Y axis.
                double delta = (position.X - m_start.X) / Sensitivity;
                m_XRotation = (XM_2PI * 2.0f * delta);

                if (m_PreviousXRotation != 0)
                {
                    m_degreesXVelocity = m_XRotation - m_PreviousXRotation;
                    if (Math.Abs(m_degreesXVelocity) > 0.1)
                    {
                        m_degreesXVelocity *= 30;
                    }
                    else
                    {
                        // allow minimal wiggle with no acceleration.
                        m_degreesXVelocity = 0;
                    }
                    // put the brakes on so movement slows down (like a flick gesture).
                    m_degreesXAcceleration = -(m_degreesXVelocity * BrakingAcceleration);
                }
                m_PreviousXRotation = m_XRotation;


                // y-rotation is rotation about the X axis.
                delta = (position.Y - m_start.Y) / Sensitivity;
                m_YRotation = (XM_2PI * 2.0f * delta);

                if (m_PreviousYRotation != 0)
                {
                    m_degreesYVelocity = m_YRotation - m_PreviousYRotation;

                    if (Math.Abs(m_degreesYVelocity) > 0.1)
                    {
                        m_degreesYVelocity *= 30;
                    }
                    else
                    {
                        // allow minimal wiggle with no acceleration.
                        m_degreesYVelocity = 0;
                    }
                    // put the brakes on so movement slows down (like a flick gesture).
                    m_degreesYAcceleration = -(m_degreesYVelocity * BrakingAcceleration);
                }
                m_PreviousYRotation = m_YRotation;

                OnChanged();
            }
        
        }

        private void OnMouseLeftButtonUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            m_tracking = false;
            StartTimer();
            if (captured)
            {
                this.container.ReleaseMouseCapture();
                captured = false;
            }
        }
        private void OnLostMouseCapture(object sender, System.Windows.Input.MouseEventArgs e)
        {
            m_tracking = false;
            StartTimer();
            captured = false;
        }

        private void StartTimer()
        {
            StopTimer();
            DispatcherTimer timer = new DispatcherTimer();
            timer.Interval = TimeSpan.FromMilliseconds(30); // 30 FPS is ok
            timer.Tick += OnTick;
            timer.Start();
            flickTimer = timer;
        }

        private void StopTimer()
        {
            DispatcherTimer timer = this.flickTimer;
            this.flickTimer = null;
            if (timer != null)
            {
                timer.Stop();
                timer.Tick -= OnTick;
            }
        }
        private void OnTick(object sender, EventArgs e)
        {
            Update();
        }

        private void OnMouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl))
            {
                // different gesture then.
                return;
            }
            StopTimer();
            m_tracking = true;
            m_YRotation = 0;
            m_XRotation = 0;
            m_PreviousXRotation = 0;
            m_PreviousYRotation = 0;
            m_start = e.GetPosition(container);
            m_StartRotation = this.Rotation;
            this.captured = this.container.CaptureMouse();
        }

        bool captured;

        private void Update()
        {
            if (!m_tracking)
            {
                // Apply the brakes on the XRotation (about the Y Axis)
                double degreesPerSecond = m_degreesXVelocity / 30; // 30fps
                m_XRotation = (m_XRotation + degreesPerSecond) % 360;
                m_degreesXVelocity += m_degreesXAcceleration;
                if (m_degreesXAcceleration <= 0 && m_degreesXVelocity <= 0)
                {
                    m_degreesXVelocity = 0;
                    m_degreesXAcceleration *= 0.9f;
                }
                else if (m_degreesXAcceleration >= 0 && m_degreesXVelocity >= 0)
                {
                    m_degreesXVelocity = 0;
                    m_degreesXAcceleration *= 0.9f;
                }

                // Apply the brakes on the YRotation (about the X Axis)
                degreesPerSecond = m_degreesYVelocity / 30; // 30fps
                m_YRotation = (m_YRotation + degreesPerSecond) % 360;
                m_degreesYVelocity += m_degreesYAcceleration;
                if (m_degreesYAcceleration <= 0 && m_degreesYVelocity <= 0)
                {
                    m_degreesYVelocity = 0;
                    m_degreesYAcceleration *= 0.9f;
                }
                else if (m_degreesYAcceleration >= 0 && m_degreesYVelocity >= 0)
                {
                    m_degreesYVelocity = 0;
                    m_degreesYAcceleration *= 0.9f;
                }

                OnChanged();

                if (m_degreesYVelocity == 0 && m_degreesXVelocity == 0)
                {
                    StopTimer();
                }
            }
        }

    }
}
