#include <behaviortree_ros/bt_action_node.h>
#include <object_deposition/CenterMoveDownAction.h>
#include <stdio.h>

using namespace BT;

class CenterMoveDownClient: public RosActionNode<object_deposition::CenterMoveDownAction>
{
public:
    CenterMoveDownClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<object_deposition::CenterMoveDownAction>(handle, name, conf) {}

    bool sendGoal(GoalType& goal) override
	{
		goal.target_object_id = 1;
		ROS_INFO("CenterMoveDownClient: sending request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override
	{
		ROS_INFO("CenterMoveDownClient: result received");
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