#include <behaviortree_ros/bt_action_node.h>
#include <common_action_service_servers/SearchRotateAction.h>

using namespace BT;

class SearchRotateClient: public RosActionNode<common_action_service_servers::SearchRotateAction>
{
public:
    SearchRotateClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<common_action_service_servers::SearchRotateAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            InputPort<int>("target_object_id"),
            InputPort<float>("yaw_iter"),
            InputPort<float>("duration"),
        };
    }

    bool sendGoal(GoalType& goal) override {
        // Get parameters from BT input ports
        if (!getInput<int>("target_object_id", goal.target_object_id)) {
            throw BT::RuntimeError("Missing required input [target_object_id]");
        }
        if (!getInput<float>("yaw_iter", goal.yaw_iter)) {
            throw BT::RuntimeError("Missing required input [yaw_iter]");
        }
        if (!getInput<float>("duration", goal.duration)) {
            throw BT::RuntimeError("Missing required input [duration]");
        }
		ROS_INFO("Sending SearchRotate Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("SearchRotate Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("SearchRotate Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("SearchRotate Action request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

    void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("SearchRotate Action halted");
    }

};