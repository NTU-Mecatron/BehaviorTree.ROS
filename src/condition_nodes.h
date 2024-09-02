#ifndef SENSOR_NODES_H
#define SENSOR_NODES_H

#include <behaviortree_ros/bt_condition_node.h>
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
class ROSObjDetectedStatus : public ROSConditionNode<std_msgs::Bool>
{
public:
    ROSObjDetectedStatus(const string& name, const NodeConfiguration& config) :
    ROSConditionNode(name, config, "/CONDITION_obj_detection_status")
    {}

protected:
    virtual bool checkCondition() const override
    {
        return _msg.data;
    }
};

#endif