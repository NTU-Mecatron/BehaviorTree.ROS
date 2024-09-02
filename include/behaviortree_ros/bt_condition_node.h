#ifndef BT_CONDITION_NODES_H
#define BT_CONDITION_NODES_H

#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <std_msgs/Bool.h>
#include <string>
#include <fstream>
#include <iostream>

using std::string;
using namespace BT;

// Generic wrapper class for ROS-based Condition Nodes
template <typename T>
class ROSConditionNode : public BT::ConditionNode 
{
public:
    ROSConditionNode(const string& name, const NodeConfiguration& config, const string& topic_name) :
    BT::ConditionNode(name, config),
    _topic_name(topic_name)
    {
        _sub = _nh.subscribe(_topic_name, 100, &ROSConditionNode::callback, this);
    }

    virtual BT::NodeStatus tick() override
    {
        ROS_INFO("%s node running", _topic_name.c_str());
        return checkCondition() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

protected:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    T _msg;
    string _topic_name;

    virtual bool checkCondition() const = 0;

    void callback(const typename T::ConstPtr& msg)
    {
        _msg = *msg;
        std::cout << "Received message on topic: " << _topic_name << std::endl;
    }
};

#endif