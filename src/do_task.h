#include <behaviortree_ros/bt_action_node.h>
#include <object_placing/ObjectDepositionAction.h>
#include <stdio.h>

using namespace BT;

class DoTaskClient: public RosActionNode<object_placing::ObjectDepositionAction>
{
public:
    DoTaskClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<object_placing::ObjectDepositionAction>(handle, name, conf) {}

    bool sendGoal(GoalType& goal) override
	{
		goal.target_object_id = 1;
		ROS_INFO("DoTaskClient: sending request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override
	{
		ROS_INFO("DoTaskClient: result received");
		bool success = res.success;
        std::string message = res.message;

        if (success == true)
        {
            return NodeStatus::SUCCESS;
        }
        else
        {
			return NodeStatus::FAILURE;
        }

	}

};