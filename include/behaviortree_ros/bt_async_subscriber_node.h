// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BEHAVIOR_TREE_BT_ASYNC_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE_BT_ASYNC_SUBSCRIBER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

using namespace std;

namespace BT
{

/**
 * Base Action to implement an Async ROS Subscriber
 */
template<class MessageT>
class AsyncRosSubscriberNode : public BT::ActionNodeBase
{
protected:

  AsyncRosSubscriberNode(ros::NodeHandle& nh, const string& name, const BT::NodeConfig& conf):
  BT::ActionNodeBase(name, conf), node_(nh) 
  { 
    const string topic = getInput<string>("topic_name").value();
    const unsigned queue_size = getInput<unsigned>("queue_size").value();
    min_fail_duration_ = getInput<unsigned>("min_fail_duration").value();
    subscriber_ = node_.subscribe( topic, queue_size, &BaseClass::callback, this );
  }

public:

  using BaseClass    = AsyncRosSubscriberNode<MessageT>;
  using MessageType  = MessageT;

  AsyncRosSubscriberNode() = delete;

  virtual ~AsyncRosSubscriberNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosSubscribe<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<string>("topic_name", "Name of the ROS topic"),
      InputPort<unsigned>("queue_size", 1, "Queue size"),
      InputPort<float>("min_fail_duration", 1.0, "Minimum duration to consider the condition fail"),
      InputPort<string>("log_msg_fail", "The message to ROS_INFO when the condition is fail")
      };
  }

  /// User must implement this method.
  virtual NodeStatus onMessageReceived(const typename MessageT::ConstPtr& msg) = 0;

  virtual void halt() override
  {
    resetStatus();
  }

  bool isNewMsgReceived() const
  {
    return is_new_msg_received_;
  }

  void resetMsgReceivedStatus()
  {
    is_new_msg_received_ = false;
  }

protected:

  ros::Subscriber subscriber_;
  ros::NodeHandle& node_;

  typename MessageT::ConstPtr msg_;
  float min_fail_duration_;
  ros::Time last_fail_time_ = ros::Time(0);
  bool is_new_msg_received_ = false;
  NodeStatus status_ = NodeStatus::RUNNING;
  bool has_logged_fail_msg_ = false;

  void callback(const typename MessageT::ConstPtr& msg)
  {
    msg_ = msg;
    is_new_msg_received_ = true;
  }

  NodeStatus tick() override
  {
    auto now = ros::Time::now();
    if( !msg_ )
    {
      if (last_fail_time_ == ros::Time(0)) 
      {
        last_fail_time_ = now;
        status_ = NodeStatus::RUNNING;
      }
      else if ((now - last_fail_time_).toSec() > min_fail_duration_)
      {
        status_ = NodeStatus::FAILURE;
      }
      else 
      {
        status_ = NodeStatus::RUNNING;
      }
      return status_;
    }
    
    return onMessageReceived(msg_);
  }
};


/// Method to register the subscriber into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterAsyncRosSubscriber(BT::BehaviorTreeFactory& factory,
                     const string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const string& name, const NodeConfig& config) {
    return make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = AsyncRosSubscriberNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ASYNC_SUBSCRIBER_NODE_HPP_