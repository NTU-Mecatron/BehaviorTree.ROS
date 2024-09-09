#include <behaviortree_ros/bt_action_node.h>
#include <object_deposition/CenterMoveUpAction.h>

using namespace BT;

class CenterMoveUpClient: public RosActionNode<object_deposition::CenterMoveUpAction>
{
public:
    CenterMoveUpClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<object_deposition::CenterMoveUpAction>(handle, name, conf) {}

    bool sendGoal(GoalType& goal) override {
		goal.target_object_id = 1;
		ROS_INFO("Sending CenterMoveUp Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("CenterMoveUp Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("CenterMoveUp Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("CenterMoveUp Action request failed: %d", static_cast<int>(failure))
        return NodeStatus::FAILURE;
    }

    virtual void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("CenterMoveUp Action halted");
    }
};