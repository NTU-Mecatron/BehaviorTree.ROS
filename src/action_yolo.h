#include <behaviortree_ros/bt_action_node.h>
#include <yolo_object_detection/OnYoloAction.h>

using namespace BT;

class YoloClient: public RosActionNode<yolo_object_detection::OnYoloAction>
{
public:
    YoloClient(ros::NodeHandle& handle, const std::string& name, const NodeConfig & conf):
		RosActionNode<yolo_object_detection::OnYoloAction>(handle, name, conf) {}

    static PortsList providedPorts()
    {
        // Define input ports for goal parameters
        return {
            InputPort<bool>("turn_on_or_off")
        };
    }

    bool sendGoal(GoalType& goal) override {
		// Get parameters from BT input ports
        bool turn_on_or_off_tmp;
        if (!getInput<bool>("turn_on_or_off", turn_on_or_off_tmp)) {
            throw BT::RuntimeError("Missing required input [turn_on_or_off]");
        }
        goal.turn_on_or_off = turn_on_or_off_tmp;
		ROS_INFO("Sending Yolo Action request");
		return true;
	}

    NodeStatus onResult( const ResultType& res) override {
        if (res.status) {
            ROS_INFO("Yolo Action returned SUCCESS");
            return NodeStatus::SUCCESS;
        } else {
            ROS_INFO("Yolo Action returned FAILURE");
			return NodeStatus::FAILURE;
        }
	}

    NodeStatus onFailedRequest(FailureCause failure) {
        ROS_INFO("Yolo Action request failed: %d", static_cast<int>(failure));
        return NodeStatus::FAILURE;
    }

    void halt() override {
        if( status() == NodeStatus::RUNNING )
        {
            action_client_->cancelGoal();
        }
        resetStatus(); 
        ROS_INFO("Yolo Action halted");
    }

};