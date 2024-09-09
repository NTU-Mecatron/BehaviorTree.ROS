#include <behaviortree_ros/bt_action_node.h>
#include <object_deposition/CenterMoveDownAction.h>

using namespace BT;

class CenterMoveDownClient: public RosActionNode<object_deposition::CenterMoveDownAction>
{
public:
    CenterMoveDownClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<object_deposition::CenterMoveDownAction>(handle, name, conf) {}

    bool sendGoal(GoalType& goal) override {
		goal.target_object_id = 1;
		ROS_INFO("Sending CenterMoveDown Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("CenterMoveDown Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("CenterMoveDown Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("CenterMoveDown Action request failed: %d", static_cast<int>(failure))
        return NodeStatus::FAILURE;
    }

    virtual void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("CenterMoveDown Action halted");
    }

};