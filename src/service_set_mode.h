#include <behaviortree_ros/bt_service_node.h>
#include <pixhawk/SetMode.h>
#include <ros/ros.h>

using namespace BT;

class SetModeClient : public RosServiceNode<pixhawk::SetMode>
{
public:
    SetModeClient( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<pixhawk::SetMode>(handle, node_name, conf) {};

    static PortsList providedPorts() {
        return  {
            InputPort<std::string>("mode")
        };
    }

    void sendRequest(RequestType& request) override {
        getInput("mode", request.mode);
        ROS_INFO("Sending SetModeClient request");
        // ros::Duration(12.0).sleep();
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        if (response.mode_sent) {
            ROS_INFO("SetModeClient request returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("SetModeClient request returned FAILURE");
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
        ROS_ERROR("SetModeClient request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    int expected_result_;

};
