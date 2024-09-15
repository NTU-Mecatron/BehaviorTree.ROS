#include <behaviortree_ros/bt_action_node.h>
#include <object_deposition/CenterAction.h>

using namespace BT;

class CenterClient: public RosActionNode<object_deposition::CenterAction>
{
public:
    CenterClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<object_deposition::CenterAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            InputPort<int>("camera_Xangle"),
            InputPort<int>("target_object_id"),
            InputPort<int>("mode"),
            InputPort<std::vector<double>>("xyz_setpoint"),
            InputPort<std::vector<double>>("xyz_margin"),
            InputPort<double>("duration")
        };
    }

    bool sendGoal(GoalType& goal) override {
		// Get parameters from BT input ports
        if (!getInput<int>("target_object_id", goal.target_object_id)) {
            throw BT::RuntimeError("Missing required input [target_object_id]");
        }
        if (!getInput<int>("mode", goal.mode)) {
            throw BT::RuntimeError("Missing required input [mode]");
        }
        if (!getInput<std::vector<double>>("xyz_setpoint", goal.xyz_setpoint)) {
            throw BT::RuntimeError("Missing required input [xyz_setpoint]");
        }
        if (!getInput<std::vector<double>>("xyz_margin", goal.xyz_margin)) {
            throw BT::RuntimeError("Missing required input [xyz_margin]");
        }
        if (!getInput<double>("duration", goal.duration)) {
            throw BT::RuntimeError("Missing required input [duration]");
        }
		ROS_INFO("Sending Center Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("Center Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("Center Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("Center Action request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

    void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("Center Action halted");
    }

};