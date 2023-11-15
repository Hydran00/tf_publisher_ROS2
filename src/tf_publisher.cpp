#include <cstdio>
#include <memory>
#include <chrono>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rate.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

class tfBroadcaster : public rclcpp::Node
{
public:
    tfBroadcaster(
        const std::string &name = "tfBroadcaster",
        const std::string &namespace_ = "",
        const rclcpp::NodeOptions &options = (rclcpp::NodeOptions()
                                                  .allow_undeclared_parameters(true)
                                                  .automatically_declare_parameters_from_overrides(true)))
        : Node(name, namespace_, options)
    {
        // this->declare_parameter("ee_frame_name", "probe");
        // this->declare_parameter("base_frame_name", "base_link");
        // this->declare_parameter("rate", 500);
        
        ee_frame_name = this->get_parameter("ee_frame_name").as_string();
        base_frame_name = this->get_parameter("base_frame_name").as_string();
        topic = this->get_parameter("topic").as_string();

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic.c_str(), 10);
        // rate = this->get_parameter("rate").as_float();

        RCLCPP_INFO(this->get_logger(), "Base frame: '%s' , Tool frame: '%s'", ee_frame_name.c_str(),  base_frame_name.c_str());
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        rclcpp::Rate pub_rate((double)500);   
        rclcpp::Clock::SharedPtr clock = this->get_clock();
        std::string warning_msg;
        while (rclcpp::ok())
        {
            broadcaster();
            pub_rate.sleep();
            //sleep for 1 second
        }
    }

    void broadcaster()
    {
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped =
                tf_buffer_->lookupTransform(this->base_frame_name, this->ee_frame_name, tf2::TimePointZero);
            // Extract position and orientation from the transform
            geometry_msgs::msg::Vector3 position = transformStamped.transform.translation;
            geometry_msgs::msg::Quaternion orientation = transformStamped.transform.rotation;
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = this->base_frame_name;
            pose.header.stamp = this->now();
            pose.pose.position.x = position.x;
            pose.pose.position.y = position.y;
            pose.pose.position.z = position.z;
            pose.pose.orientation = orientation;
            publisher_->publish(pose);
        }
        catch (tf2::TransformException &ex)
        {
            // RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
            return;
        }
    }

    // private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string ee_frame_name;
    std::string base_frame_name;
    std::string topic;
    float rate;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tfBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
