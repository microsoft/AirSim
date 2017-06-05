using LogViewer.Gestures;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace LogViewer.Controls
{
    /// <summary>
    /// Interaction logic for MeshViewer.xaml
    /// </summary>
    public partial class MeshViewer : UserControl
    {
        ModelVisual3D model;
        RotateGesture3D gesture;
        Rect3D modelBounds;
        double modelRadius;
        Point3D lookAt;

        public MeshViewer()
        {
            InitializeComponent();
            gesture = new RotateGesture3D(this);
            gesture.Changed += OnGestureChanged;
            this.Loaded += OnMeshViewerLoaded;
        }

        public Quaternion ModelAttitude
        {
            get { return (Quaternion)GetValue(ModelAttitudeProperty); }
            set { SetValue(ModelAttitudeProperty, value); }
        }

        // Using a DependencyProperty as the backing store for ModelAttitude.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty ModelAttitudeProperty =
            DependencyProperty.Register("ModelAttitude", typeof(Quaternion), typeof(MeshViewer), new PropertyMetadata(Quaternion.Identity,
                new PropertyChangedCallback(OnModelAttitudeChanged)));

        private static void OnModelAttitudeChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((MeshViewer)d).OnModelAttitudeChanged();
        }

        private void OnModelAttitudeChanged()
        {
            Update();
        }

        public Vector3D CameraPosition
        {
            get { return (Vector3D)GetValue(CameraPositionProperty); }
            set { SetValue(CameraPositionProperty, value); }
        }

        // Using a DependencyProperty as the backing store for CameraPosition.  This enables animation, styling, binding, etc...
        public static readonly DependencyProperty CameraPositionProperty =
            DependencyProperty.Register("CameraPosition", typeof(Vector3D), typeof(MeshViewer), new PropertyMetadata(new Vector3D(0,0,0), 
                new PropertyChangedCallback(OnCameraPositionChanged)));

        private static void OnCameraPositionChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            ((MeshViewer)d).OnCameraPositionChanged();
        }

        private void OnCameraPositionChanged()
        {
            Update();
        }

        private void OnMeshViewerLoaded(object sender, RoutedEventArgs e)
        {
            LoadModel();
            Update();
        }

        void Update() { 

            PerspectiveCamera camera = (PerspectiveCamera)MainViewPort.Camera;
            Vector3D position = (Vector3D)CameraPosition;
            camera.Position = (Point3D)position;

            if (this.model != null)
            {
                this.modelBounds = this.model.Content.Bounds;
                double radius = Math.Max(modelBounds.Size.X, Math.Max(modelBounds.Size.Y, modelBounds.Size.Z));
                this.modelRadius = radius;
                Point3D center = new Point3D(modelBounds.X + (modelBounds.SizeX / 2), modelBounds.Y + (modelBounds.SizeY / 2), modelBounds.Z + (modelBounds.SizeZ / 2));
                this.lookAt = center;

                if (camera.Position.X == 0 && camera.Position.Y == 0 && camera.Position.Z == 0)
                {
                    position.X = this.modelBounds.X + this.modelBounds.SizeX;
                    position.Y = this.modelBounds.Y + (this.modelBounds.SizeY / 2);
                    position.Z = this.modelBounds.Z + this.modelBounds.SizeZ;
                    position.Normalize();
                    position *= (radius * 3);
                    camera.Position = (Point3D)position;
                }

                camera.LookDirection = this.lookAt - camera.Position;

                camera.FarPlaneDistance = radius * 10;

                Quaternion rotation = this.ModelAttitude * gesture.Rotation;
                QuaternionRotation3D quaternionRotation = new QuaternionRotation3D(rotation);
                RotateTransform3D myRotateTransform = new RotateTransform3D(quaternionRotation);
                model.Transform = myRotateTransform;
                xAxis.Transform = myRotateTransform;
                yAxis.Transform = myRotateTransform;
            }

        }

        private void OnGestureChanged(object sender, EventArgs e)
        {
            this.Dispatcher.Invoke(new Action(() =>
            {
                UpdateRotation();
            }));
        }

        static DiffuseMaterial material = new DiffuseMaterial(new LinearGradientBrush(Colors.BlueViolet, Colors.Violet, 45));
        static DiffuseMaterial backMaterial = new DiffuseMaterial(new LinearGradientBrush(Colors.DarkGreen, Colors.Green, 45));


        private void LoadModel()
        {
            MeshGeometry3D mymesh = new MeshGeometry3D();

            // this resolves problems with handling of '.' versus ',' as decimal separator in the the loaded mesh.
            CultureInfo usCulture = new CultureInfo("en-US");

            using (var stream = this.GetType().Assembly.GetManifestResourceStream("LogViewer.Assets.Mesh.txt"))
            {
                using (var reader = new StreamReader(stream))
                {
                    string line = null;
                    do
                    {
                        line = reader.ReadLine();
                        if (line != null)
                        {
                            int i = line.IndexOf(',');
                            string a = line.Substring(0, i);
                            int j = line.IndexOf(',', i + 1);
                            string b = line.Substring(i + 1, j - i - 1);
                            string c = line.Substring(j + 1);

                            double x = double.Parse(a, usCulture);
                            double y = double.Parse(b, usCulture);
                            double z = double.Parse(c, usCulture);

                            mymesh.Positions.Add(new Point3D(x, y, z));
                        }
                    }
                    while (line != null);
                }
            }


            GeometryModel3D geometry = new GeometryModel3D(mymesh, material);
            geometry.BackMaterial = backMaterial;

            Model3DGroup group = new Model3DGroup();
            group.Children.Add(geometry);

            this.model = new ModelVisual3D();
            this.model.Content = group;

            this.MainViewPort.Children.Add(this.model);
        }


        private void UpdateRotation()
        {
            Quaternion rotation = this.ModelAttitude * gesture.Rotation;
            QuaternionRotation3D quaternionRotation = new QuaternionRotation3D(rotation);
            RotateTransform3D myRotateTransform = new RotateTransform3D(quaternionRotation);
            model.Transform = myRotateTransform;
            xAxis.Transform = myRotateTransform;
            yAxis.Transform = myRotateTransform;

            PerspectiveCamera camera = (PerspectiveCamera)MainViewPort.Camera;
            Vector3D position = (Vector3D)camera.Position;
            Vector3D lookDirection = this.lookAt - camera.Position;
            double length = lookDirection.Length;
            length += (gesture.Zoom * this.modelRadius / 10); // 10 clicks to travel size of model
            if (length <= 0.1)
            {
                length = 0.1;
            }
            lookDirection.Normalize();
            lookDirection *= length;
            gesture.Zoom = 0;

            camera.Position = this.lookAt - lookDirection;
        }

    }
}
