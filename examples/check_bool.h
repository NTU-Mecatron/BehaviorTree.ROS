#ifndef CONDITION_NODES_H
#define CONDITION_NODES_H

#include <behaviortree_ros/bt_subscriber_node.h>
#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/Bool.h>
#include <string>
#include <fstream>
#include <iostream>

using std::string;
using namespace BT;


// Specialized node that checks if an object is detected
class CheckBool : public RosSubscriberNode<std_msgs::Bool>
{
public:
    CheckBool(ros::NodeHandle& nh, const string& name, const NodeConfig& config) :
        RosSubscriberNode(nh, name, config) {};

    bool onMessageReceived(const std_msgs::Bool::ConstPtr& msg) override
    {
        return msg->data;
    }
};

#endif