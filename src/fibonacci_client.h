#ifndef FINBONACCI_CLIENT_H
#define FINBONACCI_CLIENT_H

#include <behaviortree_ros/bt_action_node.h>
#include <behaviortree_ros/FibonacciAction.h>

using namespace BT;

class FibonacciClient: public RosActionNode<behaviortree_ros::FibonacciAction>
{

public:
	FibonacciClient( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<behaviortree_ros::FibonacciAction>(handle, name, conf) {}

	static PortsList providedPorts()
	{
		return  
		{
			InputPort<int>("order"),
			OutputPort<int>("result") 
		};
	}

	bool sendGoal(GoalType& goal) override
	{
		if( !getInput<int>("order", goal.order) )
		{
			// abort the entire action. Result in a FAILURE
			return false;
		}
		expected_result_ = 0 + 1 + 1 + 2 + 3 + 5 + 8; // supposing order is 5
		ROS_INFO("FibonacciClient: sending request");
		return true;
	}

	NodeStatus onResult( const ResultType& res) override
	{
		ROS_INFO("FibonacciClient: result received");
		int fibonacci_result = 0;
		for( int n: res.sequence)
			fibonacci_result += n;

		if	( fibonacci_result == expected_result_)
		{
			setOutput<int>("result", fibonacci_result);
			return NodeStatus::SUCCESS;
		}
		else
		{
			ROS_ERROR("FibonacciClient replied something unexpected: %d", fibonacci_result);
			return NodeStatus::FAILURE;
		}
	}

	NodeStatus onFailedRequest(FailureCause failure) override
	{
		ROS_ERROR("FibonacciAction request failed %d", static_cast<int>(failure));
		return NodeStatus::FAILURE;
	}

	void halt() override
	{
		if( status() == NodeStatus::RUNNING )
		{
			ROS_WARN("FibonacciClient halted");
			BaseClass::halt();
		}
	}

private:
	int expected_result_;
};

#endif