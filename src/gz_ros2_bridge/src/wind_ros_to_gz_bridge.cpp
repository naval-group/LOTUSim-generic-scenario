/**
* @file wind_ros_to_gz_bridge.cpp
* @author Naval Group
* @brief  ROS2–Gazebo bridge for wind data.
* @version 0.1
* @date 2025-10-08
*
* This program and the accompanying materials are made available under the
* terms of the Eclipse Public License 2.0 which is available at:
* http://www.eclipse.org/legal/epl-2.0
*
* SPDX-License-Identifier: EPL-2.0
*
* Copyright (c) 2025 Naval Group
*/

#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/wind.pb.h>

#include <lotusim_msgs/msg/wind.hpp>

/// @brief Bridge node converting ROS 2 Wind messages into Gazebo wind messages.
class WindRos2ToGzBridge : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the bridge node and sets up publishers/subscribers.
     */
    WindRos2ToGzBridge()
        : Node("wind_ros2_to_gz_bridge")
    {
        // Declare parameters for flexibility
        this->declare_parameter<std::string>("ros_topic", "/aerialWorld/wind");
        this->declare_parameter<std::string>("gz_topic", "/world/aerialWorld/wind");

        // Get topic names from parameters
        std::string ros_topic = this->get_parameter("ros_topic").as_string();
        std::string gz_topic  = this->get_parameter("gz_topic").as_string();

        // ROS 2 subscription setup
        ros_subscription_ = this->create_subscription<lotusim_msgs::msg::Wind>(
            ros_topic, rclcpp::QoS(10),
            std::bind(&WindRos2ToGzBridge::onRosWindMsg, this, std::placeholders::_1));

        // Gazebo publisher setup
        gz_publisher_ = gz_node_.Advertise<gz::msgs::Wind>(gz_topic);

        if (!gz_publisher_)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to advertise Gazebo topic: %s", gz_topic.c_str());
            throw std::runtime_error("Gazebo publisher creation failed");
        }

        RCLCPP_INFO(this->get_logger(),
                    "Wind bridge active.\n  ROS topic: [%s]\n  Gazebo topic: [%s]",
                    ros_topic.c_str(), gz_topic.c_str());
    }

private:
    /**
     * @brief Callback to handle incoming ROS 2 wind messages and forward them to Gazebo.
     *
     * @param msg Pointer to the received ROS 2 Wind message.
     */
    void onRosWindMsg(const lotusim_msgs::msg::Wind::SharedPtr msg)
    {
        gz::msgs::Wind gz_msg;

        // Extract velocity components
        const double x = msg->linear_velocity.x;
        const double y = msg->linear_velocity.y;
        const double z = msg->linear_velocity.z;
        const bool enable = msg->enable_wind;

        // Log received data (debug level for low verbosity)
        RCLCPP_DEBUG(this->get_logger(),
            "Received ROS Wind: vel=[%.3f, %.3f, %.3f], enable=%s",
            x, y, z, enable ? "true" : "false");

        // Ensure non-zero minimal wind values to avoid Gazebo freezing issues
        auto *vel = gz_msg.mutable_linear_velocity();
        if (x == 0.0 && y == 0.0 && z == 0.0 && !enable)
        {
            vel->set_x(1e-6);
            vel->set_y(1e-6);
            vel->set_z(1e-6);
            gz_msg.set_enable_wind(true);
        }
        else
        {
            vel->set_x(x);
            vel->set_y(y);
            vel->set_z(z);
            gz_msg.set_enable_wind(true);
        }

        // Publish to Gazebo
        gz_publisher_.Publish(gz_msg);

        RCLCPP_DEBUG(this->get_logger(), "Published Gazebo wind message.");
    }

    // --- Member Variables ---
    rclcpp::Subscription<lotusim_msgs::msg::Wind>::SharedPtr ros_subscription_; ///< ROS 2 subscriber
    gz::transport::Node gz_node_;                                               ///< Gazebo transport node
    gz::transport::Node::Publisher gz_publisher_;                               ///< Gazebo publisher
};

// ============================ Main ============================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<WindRos2ToGzBridge>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("wind_ros2_to_gz_bridge"),
                     "Fatal exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
