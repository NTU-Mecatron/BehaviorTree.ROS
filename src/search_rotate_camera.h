#include <behaviortree_ros/bt_action_node.h>
#include <common_action_service_servers/SearchRotateCameraAction.h>
#include <array>
#include "conversions.h"

using namespace BT;

class SearchRotateCameraClient: public RosActionNode<common_action_service_servers::SearchRotateCameraAction>
{
public:
    SearchRotateCameraClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<common_action_service_servers::SearchRotateCameraAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            InputPort<int>("target_object_id"),
            InputPort<int>("delta_X_angle"),
            InputPort<int>("angle_setpoint"),
            InputPort<std::array<float, 3>>("xyz_setpoint"),
            InputPort<std::array<float, 3>>("xyz_margin"),
            InputPort<float>("duration"),
            InputPort<std::string>("filename"),
        };
    }

    bool sendGoal(GoalType& goal) override {
        // Get parameters from BT input ports
        if (!getInput<int>("target_object_id", goal.target_object_id)) {
            throw BT::RuntimeError("Missing required input [target_object_id]");
        }
        if (!getInput<int>("delta_X_angle", goal.delta_X_angle)) {
            throw BT::RuntimeError("Missing required input [delta_X_angle]");
        }
        if (!getInput<int>("angle_setpoint", goal.angle_setpoint)) {
            throw BT::RuntimeError("Missing required input [angle_setpoint]");
        }
        std::array<float, 3> xyz_setpoint;
        if (!getInput<std::array<float, 3>>("xyz_setpoint", xyz_setpoint)) {
            throw BT::RuntimeError("Missing required input [xyz_setpoint]");
        }
        std::copy(xyz_setpoint.begin(), xyz_setpoint.end(), goal.xyz_setpoint.begin());

        std::array<float, 3> xyz_margin;
        if (!getInput<std::array<float, 3>>("xyz_margin", xyz_margin)) {
            throw BT::RuntimeError("Missing required input [target_object_id]");
        }
        std::copy(xyz_margin.begin(), xyz_margin.end(), goal.xyz_margin.begin());
        
        if (!getInput<float>("duration", goal.duration)) {
            throw BT::RuntimeError("Missing required input [duration]");
        }
        std::string filename;
        if (!getInput<std::string>("filename", goal.filename)) {
            throw BT::RuntimeError("missing required input [filename]");
        }
		ROS_INFO("Sending SearchRotateCamera Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("SearchRotateCamera Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("SearchRotateCamera Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("SearchRotateCamera Action request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

    void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("SearchRotateCamera Action halted");
    }

};