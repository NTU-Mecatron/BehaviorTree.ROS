#include <behaviortree_ros/bt_service_node.h>
// #include <yolo_object_detection/OnYolo.h>
#include <common_msg_srv/SendFloat.h>
#include <ros/ros.h>

using namespace BT;

class CameraXAngleClient : public RosServiceNode<common_msg_srv::SendFloat>
{
public:
    CameraXAngleClient( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<common_msg_srv::SendFloat>(handle, node_name, conf) {};

    static PortsList providedPorts() {
        return  {
            InputPort<float>("x_angle")
        };
    }

    void sendRequest(RequestType& request) override {
        getInput("x_angle", request.data);
        ROS_INFO("Sending CameraXAngle request");
        // ros::Duration(12.0).sleep();
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        if (response.status) {
            ROS_INFO("CameraXAngle request returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("CameraXAngle request returned FAILURE");
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
        ROS_ERROR("CameraXAngle request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    bool expected_result_;

};
