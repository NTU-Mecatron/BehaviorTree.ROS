#ifndef PUBLISHER_NODES_H
#define PUBLISHER_NODES_H

#include <std_msgs/Bool.h>
#include <behaviortree_ros/bt_publisher_node.h>
#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <string>
#include <fstream>
#include <iostream>

// Define a ROS publisher node for Float32MultiArray
class SendBool : public RosPublisherNode<std_msgs::Bool> {
public:
    // Pass the topic name directly in the constructor
    SendBool(ros::NodeHandle& nh, const string& name, const NodeConfig& config) :
        RosPublisherNode<std_msgs::Bool>(nh, name, config) {}

    bool setMessage(std_msgs::Bool& msg) override {
        // Set the message data
        msg.data = true;
        return true;
    }
};

#endif
