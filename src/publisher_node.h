#ifndef PUBLISHER_NODES_H
#define PUBLISHER_NODES_H

#include <std_msgs/Float32MultiArray.h>
#include <behaviortree_ros/bt_publisher_node.h>
#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <string>
#include <fstream>
#include <iostream>

// Define a ROS publisher node for Float32MultiArray
class ROSPublisherNode : public ROSPublisherWrapper<std_msgs::Float32MultiArray> {
public:
    // Pass the topic name directly in the constructor
    ROSPublisherNode(const std::string& name, const BT::NodeConfiguration& config) :
        ROSPublisherWrapper<std_msgs::Float32MultiArray>(name, config, "/pixhawk/control/robotic_arm_normalized") {}

protected:
    // Override configureMessage to set the message content
    void configureMessage(std_msgs::Float32MultiArray& msg) override {
        // You can populate the Float32MultiArray with custom data here
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = 3;
        msg.layout.dim[0].stride = 1;
        msg.layout.dim[0].label = "manipulator_pos";

        // Set msg values
        msg.data.clear();
        msg.data.push_back(0.6);
        msg.data.push_back(0.7);
        msg.data.push_back(-1);
    }

    // Override configureFalseMessage to handle the timeout scenario
    void configureFalseMessage(std_msgs::Float32MultiArray& msg) override {
        // Populate with default or "false" values if needed
        msg.layout.dim.clear();
        msg.data.clear();
        msg.data.push_back(0.0);  // Example of setting the "false" data
    }
};

#endif
