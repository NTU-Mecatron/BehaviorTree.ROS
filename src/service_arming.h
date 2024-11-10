#include <behaviortree_ros/bt_service_node.h>
#include <pixhawk/CommandBool.h>
#include <ros/ros.h>

using namespace BT;

class ArmingClient : public RosServiceNode<pixhawk::CommandBool>
{
public:
    ArmingClient( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<pixhawk::CommandBool>(handle, node_name, conf) {};

    static PortsList providedPorts() {
        return  {
            InputPort<bool>("value")
        };
    }

    void sendRequest(RequestType& request) override {
        getInput("value", request.value);
        ROS_INFO("Sending ArmingClient request");
        // ros::Duration(12.0).sleep();
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        if (response.success) {
            ROS_INFO("ArmingClient request returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("ArmingClient request returned FAILURE");
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
        ROS_ERROR("ArmingClient request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    int expected_result_;

};
