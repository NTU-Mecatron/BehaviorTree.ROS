#include <behaviortree_ros/bt_action_node.h>
#include <object_deposition/CenterAction.h>

using namespace BT;

class ObjectSearch: public RosActionNode<object_deposition::ObjectSearch>
{
public:
    ObjectSearch( ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<object_deposition::ObjectSearch>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            InputPort<int>("target_object_id"),
        };
    }

    bool sendGoal(GoalType& goal) override {
		// Get parameters from BT input ports
        if (!getInput<int>("target_object_id", goal.target_object_id)) {
            throw BT::RuntimeError("Missing required input [target_object_id]");
        }
		ROS_INFO("Sending ObjectSearch Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.success) {
            ROS_INFO("ObjectSearch Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("ObjectSearch Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("ObjectSearch Action request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

    void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
        action_client_->cancelGoal();
        }
        setStatus(NodeStatus::IDLE);
        ROS_INFO("ObjectSearch Action halted");
    }

};