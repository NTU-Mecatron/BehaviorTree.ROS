#ifndef BT_PUBLISHER_NODE_H
#define BT_PUBLISHER_NODE_H

#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include <chrono>
#include <string>

using namespace BT;

template <typename T>
class ROSPublisherWrapper : public BT::SyncActionNode {
public:
    // Constructor to take the topic name directly
    ROSPublisherWrapper(const std::string& name, const BT::NodeConfiguration& config, const std::string& topic_name) :
        BT::SyncActionNode(name, config), topic_name_(topic_name)
    {
        // Set up the publisher
        pub_ = nh_.advertise<T>(topic_name_, 10);
        timer_ = nh_.createTimer(ros::Duration(1.0), &ROSPublisherWrapper::timerCallback, this);  // Timer to check tick status every second

    }

    // Override the tick method to publish the message
    BT::NodeStatus tick() override {
        for (int i=0; i<8; i++){
        last_tick_time_ = std::chrono::steady_clock::now();

        T msg;
        configureMessage(msg);  // Configure the message content

        pub_.publish(msg);
        ROS_INFO_STREAM("Published message on topic: " << topic_name_);
        ros::Duration(0.5).sleep();
    }
        return BT::NodeStatus::SUCCESS;

    }

    // Declare the ports (if needed)
    static PortsList providedPorts() {
        return {};
    }

protected:
    virtual void configureMessage(T& msg) = 0;  // This function will be implemented in the derived class

    std::string topic_name_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;
    std::chrono::steady_clock::time_point last_tick_time_;
    ros::Timer timer_;


    // Timer callback can be defined if needed
    virtual void timerCallback(const ros::TimerEvent&) {
        auto now = std::chrono::steady_clock::now();
        auto duration_since_last_tick = std::chrono::duration_cast<std::chrono::seconds>(now - last_tick_time_);
        if (duration_since_last_tick.count() > 0.1) {
            T msg;
            configureFalseMessage(msg);  // Send a "false" or default message in case of timeout
            pub_.publish(msg);
            ROS_INFO_STREAM("Published timeout message on topic: " << topic_name_);
        }
    }

    // Function to configure the "false" message (optional)
    virtual void configureFalseMessage(T& msg) {}
};

#endif
