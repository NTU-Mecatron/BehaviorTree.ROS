#include <behaviortree_ros/bt_service_node.h>
#include <yolo_object_detection/OnYolo.h>
#include <ros/ros.h>


using namespace BT;

class OnYoloClient : public RosServiceNode<yolo_object_detection::OnYolo>
{
public:

    OnYoloClient( ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfig & conf):
        RosServiceNode<yolo_object_detection::OnYolo>(handle, node_name, conf) {};

    void sendRequest(RequestType& request) override
    {
        request.turn_on = true;
        ros::Duration(12.0).sleep();

        expected_result_ = true;
        ROS_INFO("OnYolo: sending request");
    }

    NodeStatus onResponse(const ResponseType& response) override
    {
        // ros::Duration(12.0).sleep();
        ROS_INFO("OnYolo: response received");
        if (response.yolo_status == expected_result_)
        {
            return NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("AddTwoInts replied something unexpected");
            return NodeStatus::FAILURE;
        }
    }

private:
    int expected_result_;


};
