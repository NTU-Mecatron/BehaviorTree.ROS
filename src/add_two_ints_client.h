#ifndef ADD_TWO_INTS_CLIENT_H
#define ADD_TWO_INTS_CLIENT_H

#include <behaviortree_ros/bt_service_node.h>
#include <behaviortree_ros/AddTwoInts.h>

using namespace BT;

class AddTwoIntsClient : public RosServiceNode<behaviortree_ros::AddTwoInts>
{
public:

    AddTwoIntsClient( ros::NodeHandle& handle, const std::string& node_name, const NodeConfig & conf):
        RosServiceNode<behaviortree_ros::AddTwoInts>(handle, node_name, conf) {};

    static PortsList providedPorts() {
        return  {
            InputPort<int>("first_int"),
            InputPort<int>("second_int"),
            OutputPort<int>("sum") };
    }

    void sendRequest(RequestType& request) override
    {
        getInput("first_int", request.a);
        getInput("second_int", request.b);
        expected_result_ = request.a + request.b;
        ROS_INFO("AddTwoInts: sending request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
        ROS_INFO("AddTwoInts: response received");
        if( rep.sum == expected_result_)
        {
            setOutput<int>("sum", rep.sum);
            return NodeStatus::SUCCESS;
        }
        else{
            ROS_ERROR("AddTwoInts replied something unexpected: %d", rep.sum);
            return NodeStatus::FAILURE;
        }
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
        ROS_ERROR("AddTwoInts request failed %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    int expected_result_;

};

#endif