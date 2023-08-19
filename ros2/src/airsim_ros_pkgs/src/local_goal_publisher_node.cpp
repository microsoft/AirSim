//
// Created by larr-planning on 23. 8. 18.
//

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
using namespace std::chrono_literals;
namespace topic_mode {
    class LocalGoalPublisher : public rclcpp::Node {
    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        void TimerCallback(){
            static int i = 0;
            nav_msgs::msg::Odometry message;
            message.header.frame_id = "Chaser";
            message.child_frame_id = "Chaser/odom_local_ned";
            message.pose.pose.position.x = 1.5*0.02*i;
            message.pose.pose.position.y = 1.0;
            message.pose.pose.position.z = -10.0;
            message.pose.pose.orientation.w = 1.0;
            message.pose.pose.orientation.x = 0.0;
            message.pose.pose.orientation.y = 0.0;
            message.pose.pose.orientation.z = 0.0;
            publisher_->publish(message);
            i++;
        };
    public:
        LocalGoalPublisher():Node("local_goal_publisher_node"){
           publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/los_keeper/local_goal_pose",1);
           timer_ = this->create_wall_timer(20ms,std::bind(&LocalGoalPublisher::TimerCallback,this));
        }
    };

}

int main(int argc, char**argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<topic_mode::LocalGoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
