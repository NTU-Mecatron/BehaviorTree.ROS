#include <behaviortree_ros/bt_action_node.h>
#include <common_action_service_servers/CenterAction.h>
#include <array>
#include "conversions.h" 
using namespace BT;

class CenterClient: public RosActionNode<common_action_service_servers::CenterAction>
{
public:
    CenterClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<common_action_service_servers::CenterAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            // InputPort<int>("camera_Xangle"),
            InputPort<int>("target_object_id"),
            InputPort<int>("mode"),
            InputPort<std::array<float, 3>>("xyz_setpoint"),
            InputPort<std::array<float, 3>>("xyz_margin"),
            InputPort<float>("duration"),
            InputPort<int>("avoid_object_id"),
            InputPort<float>("safety_limit"),
            InputPort<float>("left_border"),
            InputPort<float>("right_border"),
            InputPort<std::string>("filename"),
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

        std::array<float, 3> xyz_setpoint;
        if (!getInput<std::array<float, 3>>("xyz_setpoint", xyz_setpoint)) {
            throw BT::RuntimeError("Missing required input [xyz_setpoint]");
        }
        std::copy(xyz_setpoint.begin(), xyz_setpoint.end(), goal.xyz_setpoint.begin());

        std::array<float, 3> xyz_margin;
        if (!getInput<std::array<float, 3>>("xyz_margin", xyz_margin)) {
            throw BT::RuntimeError("Missing required input [xyz_margin]");
        }
        std::copy(xyz_margin.begin(), xyz_margin.end(), goal.xyz_margin.begin());

        if (!getInput<float>("duration", goal.duration)) {
            throw BT::RuntimeError("Missing required input [duration]");
        }

        if (!getInput<int>("avoid_object_id", goal.avoid_object_id)) {
            throw BT::RuntimeError("Missing required input [avoid_object_id]");
        }
        if (!getInput<float>("safety_limit", goal.safety_limit)) {
            throw BT::RuntimeError("Missing required input [safety_limit]");
        }
        if (!getInput<float>("left_border", goal.left_border)) {
            throw BT::RuntimeError("Missing required input [left_border]");
        }
        if (!getInput<float>("right_border", goal.right_border)) {
            throw BT::RuntimeError("Missing required input [right_border]");
        }
        if (!getInput<float>("right_border", goal.right_border)) {
            throw BT::RuntimeError("Missing required input [right_border]");
        }
        std::string filename;
        if (!getInput<std::string>("filename", goal.filename)) {
            throw BT::RuntimeError("missing required input [filename]");
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