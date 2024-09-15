#include <behaviortree_ros/bt_service_node.h>
// #include <yolo_object_detection/OnYolo.h>
#include <common_msg_srv/SendBool.h>
#include <ros/ros.h>

using namespace BT;

class GripperClient : public RosServiceNode<common_msg_srv::SendBool>
{
public:
    GripperClient( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<common_msg_srv::SendBool>(handle, node_name, conf) {};

    static PortsList providedPorts() {
        return  {
            InputPort<bool>("close_gripper")
        };
    }

    void sendRequest(RequestType& request) override {
        getInput("close_gripper", request.request);
        ROS_INFO("Sending Gripper request");
        // ros::Duration(12.0).sleep();
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        if (response.status) {
            ROS_INFO("Gripper request returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("Gripper request returned FAILURE");
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
        ROS_ERROR("Gripper request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    bool expected_result_;

};
