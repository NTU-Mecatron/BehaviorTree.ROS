# Tutorial 3: Writing a simple BT node that calls a ROS Action

**Table of Contents**
- [Code template](#code-template)
- [Truncated template](#truncated-template)
- [Explanation](#explanation)

### Code template

For a complete example, refer to the file [fibonacci_client.h](../examples/fibonacci_client.h) in the `examples` directory.

```cpp
#ifndef YOUR_ACTION_CLIENT_H
#define YOUR_ACTION_CLIENT_H

#include <behaviortree_ros/bt_action_node.h>
#include <your_package/CustomAction.h>

using namespace BT;

class MyActionClient: public RosActionNode<your_package::CustomAction>
{

public:
	MyActionClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<behaviortree_ros::CustomAction>(handle, name, conf) {}

	static PortsList providedPorts()
	{
		return  
		{
			InputPort<int>("port_name_1"),
			OutputPort<int>("port_name_2") 
		};
	}

	bool sendGoal(GoalType& goal) override
	{
		if( !getInput<int>("port_name_1", goal.data) )
		{
			// abort the entire action. Result in a FAILURE
			return false;
		}
		ROS_INFO("MyActionClient: sending request");
		return true;
	}

	NodeStatus onResult( const ResultType& res) override
	{
		ROS_INFO("MyActionClient: result received");

		if	( res.result == expected_result_)
		{
			return NodeStatus::SUCCESS;
		}
		else
		{
			return NodeStatus::FAILURE;
		}
	}

	NodeStatus onFailedRequest(FailureCause failure) override
	{
		ROS_ERROR("CustomAction request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}

	void halt() override
	{
		if( status() == NodeStatus::RUNNING )
		{
			ROS_WARN("MyActionClient halted");
			BaseClass::halt();
		}
	}

private:
    // Add your private variables and methods here if any
    int expected_result_;
};

#endif
```

### Truncated template

```cpp
#ifndef YOUR_ACTION_CLIENT_H
#define YOUR_ACTION_CLIENT_H

#include <behaviortree_ros/bt_action_node.h>
#include <your_package/CustomAction.h>

using namespace BT;

class MyActionClient: public RosActionNode<your_package::CustomAction>
{

public:
	MyActionClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<behaviortree_ros::CustomAction>(handle, name, conf) {}

	bool sendGoal(GoalType& goal) override
	{
		goal.goal = true;
		return true;
	}

	NodeStatus onResult( const ResultType& res) override
	{
		ROS_INFO("MyActionClient: result received");

		if	( res.result == true)
			return NodeStatus::SUCCESS;
		else
			return NodeStatus::FAILURE;
	}

};

#endif
```

> Tip: to quickly edit the template, you can highlight the template word, right click to select `Change All Occurrences` and then type the appropriate names.

### Explanation

The following methods are to be implemented in the derived class:

- `sendGoal` (compulsory): This method is called when the action node is executed. It should fill the `goal` object with the data that needs to be sent to the server. If the method returns `false`, the action is aborted and the node returns `FAILURE`. Again, you do not need to call the action server here. The base class already took care of that.

- `onResult` (compulsory): This method is called when the action server sends a result. The `res` object contains the data sent by the server. You should check the result and return `NodeStatus::SUCCESS` or `NodeStatus::FAILURE` accordingly.

- `onFailedRequest` (optional): This method is called when the action server fails to execute the action. You can use this method to log the error and return `NodeStatus::FAILURE`.

- `halt` (optional): This method is called when the action node is interrupted. You can use this method to clean up resources or send a cancel request to the server. If you override this method, you should call the base class implementation at the end.

Most of the times, you will just need the truncated template to send an action call to start an action.