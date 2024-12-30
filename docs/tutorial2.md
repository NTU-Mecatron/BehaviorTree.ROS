# Tutorial 2: Writing a simple BT node that calls a ROS Service

**Table of Contents**

- [Code template](#code-template)
- [Truncated code template](#truncated-code-template)
- [Explanation](#explanation)

### Code template

For a complete example, refer to the file [add_two_ints_client.h](../examples/add_two_ints_client.h) in the `examples` directory.

```cpp
#ifndef YOUR_CLASS_NAME_H
#define YOUR_CLASS_NAME_H

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros/bt_service_node.h>
#include <your_package/CustomService.h>

using namespace BT;

class MyServiceClient : public RosServiceNode<your_package::CustomService>
{
public:

    MyServiceClient( ros::NodeHandle& handle, const std::string& node_name, const NodeConfig& conf):
        RosServiceNode<your_package::CustomService>(handle, node_name, conf) 
        {
            // Add your initialization code here and set default values if needed
            // You can also leave it empty
            my_variable = 0;
        };

    static PortsList providedPorts() 
    {
        return  
        {
            InputPort<int>("port_name_1"),
            OutputPort<int>("port_name_2") 
            // Add more ports if you'd like
        };
    }

    void sendRequest(RequestType& request) override
    {
        getInput("port_name_1", request.data);
        ROS_INFO("CustomService: sending request");
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
        ROS_INFO("CustomService: response received");
        // You can add what else you want to do with the response here
        return NodeStatus::SUCCESS;
    }

    NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override
    {
        ROS_ERROR("CustomService request failed %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

private:
    // Add your private variables and methods here if any
    int my_variable;

};

#endif
```

### Truncated code template

```cpp
#ifndef YOUR_CLASS_NAME_H
#define YOUR_CLASS_NAME_H

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_ros/bt_service_node.h>
#include <your_package/CustomService.h>

using namespace BT;

class MyServiceClient : public RosServiceNode<your_package::CustomService>
{
public:

    MyServiceClient( ros::NodeHandle& handle, const std::string& node_name, const NodeConfig& conf):
        RosServiceNode<your_package::CustomService>(handle, node_name, conf) {};

    void sendRequest(RequestType& request) override
    {
        request.request = true;
    }

    NodeStatus onResponse(const ResponseType& rep) override
    {
        ROS_INFO("CustomService: response received");
        // You can add what else you want to do with the response here
        return NodeStatus::SUCCESS;
    }

};

#endif
```

> Tip: to quickly edit the template, you can highlight the template word, right click to select `Change All Occurrences` and then type the appropriate names.

### Explanation

The following methods are to be implemented in the derived class:

- `sendRequest` (compulsory): This method is called when the BT node is executed. You can use this method to fill the request message with the data you want to send to the service. However, you *do not need to actually call any function to send the data* as it is already implement in the `tick` function of the base class.

- `onResponse` (compulsory): This method is called when the service response is received. You can use this method to process the response data and return the appropriate status. For example, the return data can be true or false, indicating whether the request was successfully carried out or not, and then return `NodeStatus::SUCCESS` or `NodeStatus::FAILURE` respectively.

- `providedPorts` (optional): This method is used to define the input and output ports of the BT node. You can add as many ports as you need. Note: the base class already added a port called `service_name` which is used to specify the name of the ROS service you want to call, so most of the times you do not need to add this function.

- `onFailedRequest` (optional): This method is called when you cannot call the service. You can use this method to log the error and return `NodeStatus::FAILURE`.

Most of the times, you will just need the truncated template to send a service call for something to be done.