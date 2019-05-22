namespace airsim_ros
{
    struct XYZYaw
    {
        double x;
        double y;
        double z;
        double yaw;
    };

    class DynamicConstraints
    {
    public:
        double max_vel_horz_abs; // meters/sec
        double max_vel_vert_abs;
        double max_yaw_rate_degrees;

        DynamicConstraints():
            max_vel_horz_abs(1.0),
            max_vel_vert_abs(0.5),
            max_yaw_rate_degrees(10.0)
            {}

        bool load_from_rosparams(const ros::NodeHandle& nh)
        {
            bool found = true;

            found = found && nh.getParam("max_vel_horz_abs", max_vel_horz_abs);
            found = found && nh.getParam("max_vel_vert_abs", max_vel_vert_abs);
            found = found && nh.getParam("max_yaw_rate_degrees", max_yaw_rate_degrees);

            return found;
        }
    };

    // todo better naming
    class DynamicConstraintsVelAccYaw
    {
    public:
        double max_vel;
        double max_acc;
        double max_yaw_rate_degrees;

        DynamicConstraintsVelAccYaw():
            max_vel(1.0),
            max_acc(2.0),
            max_yaw_rate_degrees(10.0)
            {}

        bool load_from_rosparams(const ros::NodeHandle& nh)
        {
            bool found = true;

            found = found && nh.getParam("max_vel", max_vel);
            found = found && nh.getParam("max_acc", max_acc);
            found = found && nh.getParam("max_yaw_rate_degrees", max_yaw_rate_degrees);

            return found;
        }
    };
}