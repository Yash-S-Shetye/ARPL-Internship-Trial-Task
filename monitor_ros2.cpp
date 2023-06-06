#include "cpp_pubsub/ros_compat.h"
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/exceptions.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>

// Declaring the global variables
std::string status;
std::string mav_name;
std::string yaml_file;
float frequency, threshold_1, threshold_2;

// Main class
class TopicMonitor {
public:
    rclcpp::Node::SharedPtr node_;
    rclcpp::SubscriptionBase::SharedPtr subscriber;
    std::string topic;
    std::string alias;
    std::string msg_type;
    int msg_count;
    float frequency;
    float target_frequency, threshold_1, threshold_2;

    // Constructor function
    TopicMonitor(rclcpp::Node::SharedPtr node, const std::string& topic, const std::string& alias, const std::string& msg_type, float target_frequency)
        : node_(node), topic(topic), alias(alias), msg_type(msg_type), msg_count(0), frequency(0), target_frequency(target_frequency) {}

    template<typename MessageType>

    // Callback function
    void callback(const typename MessageType::SharedPtr msg) {
        msg_count++;
        //ROS_INFO("Callback");
        //ROS_INFO("%d", msg_count);
    }

    // Function to update the frequency
    float update_frequency() {
        frequency = msg_count;
        msg_count = 0;
        return frequency;
    }

    // Function to get the status
    std::string get_status() {
        if (frequency >= target_frequency * threshold_1) {
            return "  ✅      |";
        } else if (frequency >= target_frequency * threshold_2) {
            return "  ⚠️      |";
        } else {
            return "  ❌      |";
        }
    }
};

// Function to display table of frequencies
void display_table(const std::vector<TopicMonitor>& monitors, const std::string& status) {
    // Define column widths
    const std::streamsize alias_width = 15;
    const std::streamsize frequency_width = 10;
    const std::streamsize status_width = 19;

    // Print top border
    std::cout << std::setw(alias_width) << std::setfill('=') << ""
              << std::setw(frequency_width) << ""
              << std::setw(status_width) << "" << std::endl;

    // Print table header
    std::cout << std::setfill(' ')
              << "| " << std::setw(7)
              << std::setw(alias_width) << std::left << "Alias" << " | "
              << std::setw(frequency_width) << std::left << "Hz" << " | "
              << std::setw(status_width) << std::left << "Status 💡 |"
              << std::endl;

    // Print header separator
    std::cout << std::setw(alias_width) << std::setfill('-') << ""
              << std::setw(frequency_width) << ""
              << std::setw(status_width) << "" << std::endl;

    // Print table rows
    for (const auto& monitor : monitors) {
        std::cout << "| " << std::setw(7) << std::setfill(' ')
                  << std::setw(alias_width) << std::left << monitor.alias << " | "
                  << std::setw(frequency_width) << std::left << monitor.frequency << " | "
                  << std::setw(status_width) << std::left << status
                  << std::endl;
    }

    // Print bottom border
    std::cout << std::setw(alias_width) << std::setfill('=') << ""
              << std::setw(frequency_width) << ""
              << std::setw(status_width) << "" << std::endl;
}

// Function to monitor all the frequencies
void monitor_frequency() {

    //rclcpp::NodeOptions options;
    auto node = std::make_shared<rclcpp::Node>("topic_monitor");

    std::string package_path = ament_index_cpp::get_package_share_directory("cpp_pubsub");     // Specifying the ros package path
    std::string yaml_file = package_path + "/config/monitor.yaml"; // Specifying the location of yaml file

    // Load data from yaml file
    YAML::Node topicsNode = YAML::LoadFile(yaml_file);
    /*if (!ROS_GET_PARAMETER("yaml_file", yaml_file)) {
        ROS_ERROR("Failed to retrieve 'yaml_file' parameter");
        return 1;
    }*/

    threshold_1 = topicsNode["threshold_1"].as<float>();
    threshold_2 = topicsNode["threshold_2"].as<float>();

    // Get the MAV_NAME from user
    /*if (!ROS_GET_PARAMETER("mav_name", mav_name)) {
        ROS_ERROR("Failed to retrieve 'mav_name' parameter");
        return 1;
    }*/

    std::string mav_name = std::getenv("MAV_NAME");

    //else {
    // Create a monitor for each topic
    std::vector<TopicMonitor> monitors;
    for (const auto& msg_types : topicsNode["entries"]) {
        std::string msg_type = msg_types["msg_type"].as<std::string>();

        for (const auto& topicNode : msg_types["topics"]) {
            std::string topic = topicNode["topic"].as<std::string>();
            std::string alias = topicNode["alias"].as<std::string>();
            float target_frequency = topicNode["target_frequency"].as<float>();
            topic = "/" + mav_name + topic;
            TopicMonitor monitor(node, topic, alias, msg_type, target_frequency);
            monitors.push_back(monitor);
        }
    }

    // Loop to keep displaying table of frequencies until user ends the program
    //rclcpp::Rate rate(1);
    //while (ROS_OK()) {
        for (auto& monitor : monitors) {
            if (monitor.msg_type == "sensor_msgs/msg/CompressedImage") {
                monitor.subscriber = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::CompressedImage, monitor.topic, 1000, [&monitor](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                monitor.callback<sensor_msgs::msg::CompressedImage>(msg);});
            }
            else if (monitor.msg_type == "sensor_msgs/msg/Imu") {
                monitor.subscriber = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::Imu, monitor.topic, 1000, [&monitor](const sensor_msgs::msg::Imu::SharedPtr msg) {
                monitor.callback<sensor_msgs::msg::Imu>(msg);});
            }else if (monitor.msg_type == "sensor_msgs/msg/NavSatFix") {
                monitor.subscriber = ROS_CREATE_SUBSCRIBER(sensor_msgs::msg::NavSatFix, monitor.topic, 1000, [&monitor](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
                monitor.callback<sensor_msgs::msg::NavSatFix>(msg);});
            } else if (monitor.msg_type == "nav_msgs/msg/Odometry") {
                monitor.subscriber = ROS_CREATE_SUBSCRIBER(nav_msgs::msg::Odometry, monitor.topic, 1000, [&monitor](const nav_msgs::msg::Odometry::SharedPtr msg) {
                monitor.callback<nav_msgs::msg::Odometry>(msg);});
            } else if (monitor.msg_type == "geometry_msgs/msg/TwistStamped") {
                monitor.subscriber = ROS_CREATE_SUBSCRIBER(geometry_msgs::msg::TwistStamped, monitor.topic, 1000, [&monitor](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                monitor.callback<geometry_msgs::msg::TwistStamped>(msg);});
            }else {
                ROS_ERROR("Unsupported msg_type: %s", monitor.msg_type.c_str());
            }
        }

        rclcpp::Rate rate(1); 
        while(ROS_OK()) {
            for(auto& monitor : monitors) {
                float frequency = monitor.update_frequency();
                status = monitor.get_status();
                ROS_SPIN_ONCE();
            }
            rate.sleep();

        // Calling display table function
        display_table(monitors, status);

        std::cout << std::endl;

    //} 
    }
}

// Main function
int main(int argc, char** argv) {
    ROS_CREATE_NODE("topic_monitor");
    //try {
    monitor_frequency();
    //} catch (const rclcpp::Exception& e) {
        //RCLCPP_ERROR_STREAM(rclcpp::get_logger("topic_monitor"), "ROS exception: " << e.what());
    //}

    return 0;
}
