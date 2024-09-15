#include <behaviortree_ros/bt_service_node.h>
#include <common_msg_srv/SendBool.h>
#include <ros/ros.h>

using namespace BT;

// TODO: Write service file for MoveArm ROS Service 
class MoveArm : public RosServiceNode<common_msg_srv::SendBool>
{
public:
    MoveArm( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<common_msg_srv::SendBool>(handle, node_name, conf) {};

    static PortsList providedPorts() {
        return  {
            InputPort<int>("theta1"),
            InputPort<int>("theta2")
        };
    }

    void sendRequest(RequestType& request) override {
        if (!getInput<int>("theta1", goal.theta1)) {
            throw BT::RuntimeError("Missing required input [theta1]");
        }
        if (!getInput<int>("theta2", goal.theta2)) {
            throw BT::RuntimeError("Missing required input [theta2]");
        }
        ROS_INFO("Sending MoveArm request");
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        if (response.status) {
            ROS_INFO("MoveArm request returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("MoveArm request returned FAILURE");
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
        ROS_ERROR("MoveArm request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

};
