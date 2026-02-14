#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace mppi {

/**
 * @brief Publisher for MPPI critic debugging statistics
 * 
 * Publishes diagnostic messages containing information about which critics
 * are affecting trajectory costs and by how much.
 */
class DebugPublisher {
public:
    using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

    DebugPublisher() = default;

    /**
     * @brief Configure the debug publisher
     * 
     * @param node ROS2 node to use for publishing
     * @param topic_name Topic to publish debug info on
     * @param enable Whether publishing is enabled
     * @param pub_rate_hz Publishing rate in Hz (0 = no rate limiting)
     */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
        const std::string &topic_name,
        bool enable,
        double pub_rate_hz
    ) {
        node_ = node;
        enable_ = enable;
        setPubRate(pub_rate_hz);

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.best_effort();
        qos.durability_volatile();

        pub_ = node_->create_publisher<DiagnosticArray>(topic_name, qos);
        last_pub_time_ = node_->now();
    }

    /**
     * @brief Enable or disable publishing
     */
    void enable(bool en) { enable_ = en; }

    /**
     * @brief Set the publishing rate
     * 
     * @param hz Frequency in Hz (0 or negative = no rate limiting)
     */
    void setPubRate(double hz) {
        publish_period_ = rclcpp::Duration::from_seconds(
            hz <= 0.0 ? 0.0 : (1.0 / hz)
        );
    }

    /**
     * @brief Publish critic statistics if enough time has elapsed
     * 
     * @param critic_names Names of all critics
     * @param changed Whether each critic changed costs
     * @param delta_sum Sum of cost deltas for each critic
     * @param delta_abs_sum Sum of absolute cost deltas for each critic
     * @param fail_flag Whether trajectory evaluation failed
     */
    void publishIfDue(
        const std::vector<std::string> &critic_names,
        const std::vector<bool> &changed,
        const std::vector<float> &delta_sum,
        const std::vector<float> &delta_abs_sum,
        bool fail_flag
    ) {
        if (!enable_ || !pub_ || !node_) {
            return;
        }

        const auto now = node_->now();
        if ((now - last_pub_time_) < publish_period_) {
            return;
        }

        last_pub_time_ = now;

        DiagnosticArray msg;
        msg.header.stamp = now;

        using namespace diagnostic_msgs::msg;

        // Add summary status
        DiagnosticStatus summary;
        summary.name = "mppi_critic_debug/summary";
        summary.level = DiagnosticStatus::OK;
        summary.message = fail_flag ? "Something_failed" : "No_failures";
        summary.values.reserve(2);

        KeyValue kv;
        kv.key = "fail_flag";
        kv.value = fail_flag ? "1" : "0";
        summary.values.push_back(kv);

        kv.key = "num_critics";
        kv.value = std::to_string(critic_names.size());
        summary.values.push_back(kv);

        msg.status.push_back(std::move(summary));

        // Add individual critic statuses
        const size_t num_critics = critic_names.size();
        msg.status.reserve(msg.status.size() + num_critics);

        for (size_t i = 0; i < num_critics; ++i) {
            DiagnosticStatus critic_status;
            critic_status.name = "mppi_critic_debug/" + critic_names[i];
            critic_status.level = DiagnosticStatus::OK;
            critic_status.message = 
                (i < changed.size() && changed[i]) ? "changed" : "no_changes";

            critic_status.values.reserve(3);

            // Delta sum (signed)
            KeyValue kv_delta;
            kv_delta.key = "delta_sum";
            kv_delta.value = (i < delta_sum.size()) ? 
                std::to_string(delta_sum[i]) : "0.0";
            critic_status.values.push_back(kv_delta);

            // Delta absolute sum
            KeyValue kv_abs;
            kv_abs.key = "delta_abs_sum";
            kv_abs.value = (i < delta_abs_sum.size()) ? 
                std::to_string(delta_abs_sum[i]) : "0.0";
            critic_status.values.push_back(kv_abs);

            // Changed flag
            KeyValue kv_changed;
            kv_changed.key = "changed";
            kv_changed.value = (i < changed.size() && changed[i]) ? "1" : "0";
            critic_status.values.push_back(kv_changed);

            // FIX: Push critic_status, not summary!
            msg.status.push_back(std::move(critic_status));
        }

        pub_->publish(std::move(msg));
    }

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;    
    rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_;
    bool enable_ = false;
    rclcpp::Duration publish_period_{rclcpp::Duration::from_seconds(0.2)};
    rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace mppi
