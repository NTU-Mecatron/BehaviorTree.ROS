#ifndef SENSOR_NODES_H
#define SENSOR_NODES_H

#include <ros/ros.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/blackboard.h"
#include <std_msgs/Bool.h>
#include <string>
#include <fstream>
#include <iostream>

using std::string;
using namespace BT;


class ROSObjDetectedStatus : public BT::ConditionNode 
{
public:
	ROSObjDetectedStatus(const string& name, const NodeConfiguration& config) :
	BT::ConditionNode(name, config)
	{
		_obj_detection_state_sub = _nh.subscribe("/CONDITION_obj_detection_status", 100, &ROSObjDetectedStatus::ObjDetectedCallback, this );
    }

    BT::NodeStatus tick() override
	{
		ROS_INFO ("Obj detection node running");
		return (_obj_detection_state_msg.data == true) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
	}

	static BT::PortsList providedPorts()
	{
		return {};
	}
	
private:
	ros::NodeHandle _nh;
	ros::Subscriber _obj_detection_state_sub;
	std_msgs::Bool _obj_detection_state_msg;

	void ObjDetectedCallback(const std_msgs::Bool::ConstPtr& msg)
	{   
        std::cout << "getting obj detection status" <<std::endl;
		_obj_detection_state_msg = *msg;
	}
};

#endif