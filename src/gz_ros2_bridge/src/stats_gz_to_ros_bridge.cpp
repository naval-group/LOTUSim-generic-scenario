/**
* @file stats_gz_to_ros2_bridge.cpp
* @author Naval Group
* @brief ROS2–Gazebo bridge for world statistics.
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
#include <gz/msgs/world_stats.pb.h>

#include <lotusim_msgs/msg/sim_stats.hpp>
#include <builtin_interfaces/msg/duration.hpp>

/// @brief Node bridging Gazebo world statistics to ROS 2.
class StatsGzToRos2Bridge : public rclcpp::Node
{
public:
    /**
     * @brief Constructs the bridge node.
     * Initializes the ROS 2 publisher and subscribes to the Gazebo topic.
     */
    StatsGzToRos2Bridge()
    : Node("gz_to_ros_stats_bridge")
    {
        // QoS Configuration
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default));
        qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
           .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
           .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST)
           .keep_last(10);

        // ROS 2 publisher setup
        constexpr auto kRosTopic = "/defenseScenario/sim_stats";
        ros_publisher_ = this->create_publisher<lotusim_msgs::msg::SimStats>(kRosTopic, qos);

        // Gazebo subscription setup
        constexpr auto kGzTopic = "world/defenseScenario/stats";
        if (!gz_node_.Subscribe(kGzTopic, &StatsGzToRos2Bridge::OnGzStats, this))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to Gazebo topic: %s", kGzTopic);
            throw std::runtime_error("Failed to subscribe to Gazebo topic");
        }

        RCLCPP_INFO(this->get_logger(), "Bridging Gazebo topic [%s] → ROS topic [%s]", kGzTopic, kRosTopic);
    }

private:
    /**
     * @brief Callback triggered when a new Gazebo world stats message is received.
     *
     * @param gz_msg Gazebo world statistics message.
     */
    void OnGzStats(const gz::msgs::WorldStatistics &gz_msg)
    {
        lotusim_msgs::msg::SimStats ros_msg;

        // Map data fields
        ros_msg.iterations = gz_msg.iterations();
        ros_msg.real_time_factor = gz_msg.real_time_factor();

        // Extract step size (gz::msgs::Time → builtin_interfaces/msg/Duration)
        ros_msg.step_size.sec = static_cast<int32_t>(gz_msg.step_size().sec());
        ros_msg.step_size.nanosec = static_cast<uint32_t>(gz_msg.step_size().nsec());

        ros_publisher_->publish(ros_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published sim stats: iterations=%lu, RTF=%.3f",
                     ros_msg.iterations, ros_msg.real_time_factor);
    }

    // --- Member Variables ---
    rclcpp::Publisher<lotusim_msgs::msg::SimStats>::SharedPtr ros_publisher_; ///< ROS2 publisher
    gz::transport::Node gz_node_; ///< Gazebo Transport node
};

// ============================ Main ============================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<StatsGzToRos2Bridge>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("gz_to_ros_stats_bridge"),
                     "Exception in main(): %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
