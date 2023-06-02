// Including all the required libraries

#include "ros/ros.h"
#include "ros/package.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "iomanip"
#include "std_msgs/String.h"
#include "yaml-cpp/yaml.h"

// Declaring the global variables
std::string status;
std::string mav_name;
std::string yaml_file;
float frequency, threshold_1, threshold_2;

// Main class
class TopicMonitor {
    public:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;
    std::string topic;
    std::string alias;
    std::string msg_type;
    int msg_count;
    float frequency, target_frequency;
    // Constructor function
    TopicMonitor(const std::string& topic, const std::string& alias, const std::string& msg_type, float target_frequency)
        : topic(topic), alias(alias), msg_type(msg_type), msg_count(0), frequency(0), target_frequency(target_frequency) {
    }

    template<typename MessageType>
    // Callback function
    void callback(const typename MessageType::ConstPtr& data) {
        msg_count++;
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
void display_table(std::vector<TopicMonitor> monitors, std::string status) {
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
int monitor_frequency() {
    ros::NodeHandle nh("~");
    // Get yaml file from launcch file and check if it's accesed properly
    if (!nh.getParam("yaml_file", yaml_file)) {
        ROS_ERROR("Failed to retrieve 'yaml_file' parameter");
        return 1;
    }
  
    // Loading the yaml file
    YAML::Node topicsNode = YAML::LoadFile(yaml_file);
    threshold_1 = topicsNode["threshold_1"].as<float>();
    threshold_2 = topicsNode["threshold_2"].as<float>();

     // Get the MAV_NAME defined as environent variable
    if(!nh.getParam("mav_name", mav_name)) {
        ROS_ERROR("Failed to retrieve 'parameter_name' parameter");
        return 1;
    }
    else {
    // Create a monitor for each topic
    std::vector<TopicMonitor> monitors;
    for (const auto& msg_types : topicsNode["entries"]) {
        std::string msg_type = msg_types["msg_type"].as<std::string>();

        for(const auto& topicNode : msg_types["topics"]) {
            std::string topic = topicNode["topic"].as<std::string>();
            std::string alias = topicNode["alias"].as<std::string>();
            float target_frequency = topicNode["target_frequency"].as<float>();
            topic = "/" + mav_name + topic;
            TopicMonitor monitor(topic, alias, msg_type, target_frequency);
            monitors.push_back(monitor);
        }

    }

    // Update the frequency every second
    ros::Rate rate(1);

    // Loop to keep displaying table of frequencies till user ends program
    while (ros::ok()) {

        ros::spinOnce();

        std::cout << "\033[2J\033[1;1H"; // Clear the screen

        for (auto& monitor : monitors) {
            if(monitor.msg_type == "sensor_msgs/CompressedImage") {
                monitor.subscriber = nh.subscribe(monitor.topic, 1000, &TopicMonitor::callback<sensor_msgs::CompressedImage>, &monitor);
            }
            else if(monitor.msg_type == "sensor_msgs/Imu") {
                monitor.subscriber = nh.subscribe(monitor.topic, 1000, &TopicMonitor::callback<sensor_msgs::Imu>, &monitor);
            }
            else if(monitor.msg_type == "sensor_msgs/NavSatFix") {
                monitor.subscriber = nh.subscribe(monitor.topic, 1000, &TopicMonitor::callback<sensor_msgs::NavSatFix>, &monitor);
            }
            else if(monitor.msg_type == "nav_msgs/Odometry") {
                monitor.subscriber = nh.subscribe(monitor.topic, 1000, &TopicMonitor::callback<nav_msgs::Odometry>, &monitor);
            }else if(monitor.msg_type == "geometry_msgs/TwistStamped") {
                monitor.subscriber = nh.subscribe(monitor.topic, 1000, &TopicMonitor::callback<geometry_msgs::TwistStamped>, &monitor);
            }
            else {
                ROS_ERROR("Unsupported topic: %s", monitor.topic.c_str());
            }

            float frequency = monitor.update_frequency();
            status = monitor.get_status();
        }
        
        // Calling display table function
        display_table(monitors, status);

        std::cout << std::endl;

        rate.sleep();
    }}
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "topic_monitor");
    try {
        monitor_frequency();
    } catch (const ros::Exception& e) {
        ROS_ERROR_STREAM("ROS exception: " << e.what());
    }

    return 0;
}
