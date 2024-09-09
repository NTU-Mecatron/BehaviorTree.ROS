#include <behaviortree_ros/bt_service_node.h>
#include <yolo_object_detection/OnYolo.h>
#include <ros/ros.h>

using namespace BT;

class OnYoloClient : public RosServiceNode<yolo_object_detection::OnYolo>
{
public:
    OnYoloClient( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<yolo_object_detection::OnYolo>(handle, node_name, conf) {};

    void sendRequest(RequestType& request) override {
        ROS_INFO("Sending OnYolo request");
        request.turn_on = true;
        ros::Duration(12.0).sleep();
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        if (response.yolo_status) {
            ROS_INFO("OnYolo request returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_ERROR("OnYolo request returned FAILURE");
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override {
        ROS_ERROR("OnYolo request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    int expected_result_;

};
