#include <behaviortree_ros/bt_action_node.h>
#include <common_action_service_servers/BlindMovementAction.h>

using namespace BT;

class BlindMovementClient: public RosActionNode<common_action_service_servers::BlindMovementAction>
{
public:
    BlindMovementClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<common_action_service_servers::BlindMovementAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            InputPort<float>("x"),
            InputPort<float>("y"),
            InputPort<float>("z"),
            InputPort<float>("yaw"),
            InputPort<float>("duration")
        };
    }

    bool sendGoal(GoalType& goal) override {
		// Get parameters from BT input ports
        if (!getInput<float>("x", goal.x)) {
            throw BT::RuntimeError("Missing required input [x]");
        }
        if (!getInput<float>("y", goal.y)) {
            throw BT::RuntimeError("Missing required input [y]");
        }
        if (!getInput<float>("z", goal.z)) {
            throw BT::RuntimeError("Missing required input [z]");
        }
        if (!getInput<float>("yaw", goal.yaw)) {
            throw BT::RuntimeError("Missing required input [yaw]");
        }
        if (!getInput<float>("duration", goal.duration)) {
            throw BT::RuntimeError("Missing required input [duration]");
        }
		ROS_INFO("Sending BlindMovement Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("BlindMovement Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("BlindMovement Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("BlindMovement Action request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

    void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("BlindMovement Action halted");
    }

};