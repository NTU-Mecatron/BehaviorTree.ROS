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

#ifndef BEHAVIOR_TREE_BT_ASYNC_PUBLISHER_NODE_HPP_
#define BEHAVIOR_TREE_BT_ASYNC_PUBLISHER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

/**
 * Base Action to implement a ROS Publisher
 */

template<class MessageT>
class AsyncRosPublisherNode : public BT::ActionNodeBase
{
protected:

  AsyncRosPublisherNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfig& conf):
  BT::ActionNodeBase(name, conf), node_(nh) 
  {
    const std::string topic = getInput<std::string>("topic_name").value();
    const unsigned queue_size = getInput<unsigned>("queue_size").value();
    publisher_ = node_.advertise<MessageT>(topic, queue_size);

    period_ = 1.0 / getInput<int>("publish_frequency").value();
    timeout_ = getInput<float>("publish_timeout").value();
  }

public:

  using BaseClass    = AsyncRosPublisherNode<MessageT>;
  using MessageType  = MessageT;

  AsyncRosPublisherNode() = delete;

  virtual ~AsyncRosPublisherNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosPublisher<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("topic_name", "Name of the ROS topic"),
      InputPort<unsigned>("queue_size", 1, "Queue size"),
      InputPort<int>("publish_frequency", 1, "Frequency in Hz"),
      InputPort<float>("publish_timeout", 1, "Duration to keep publishing (secs). Set to -1 to publish forever and return RUNNING"),
      };
  }

  /// User must implement this method.
  virtual bool setMessage(MessageT& msg) = 0;

  virtual void halt() override
  {
    resetStatus();
  }

protected:

  ros::Publisher publisher_;
  ros::NodeHandle& node_;
  float period_;
  float timeout_;
  ros::Time start_time_ = ros::Time(0);
  ros::Time last_time_ = ros::Time(0);

  NodeStatus tick() override
  {
    if (start_time_ == ros::Time(0))
      start_time_ = ros::Time::now();
    
    if (timeout_ > 0 && (ros::Time::now() - start_time_).toSec() > timeout_)
    {
      start_time_ = ros::Time(0);
      last_time_ = ros::Time(0);
      return NodeStatus::SUCCESS;
    }

    MessageT msg_;
    if( setMessage(msg_))
    {
      if ((ros::Time::now() - last_time_).toSec() > period_)
      {
        publisher_.publish(msg_);
        last_time_ = ros::Time::now();
      }
      return NodeStatus::RUNNING;
    }
    else
    {
      cout << "Failed to publish message" << endl;
      return NodeStatus::FAILURE;
    }
  }
};

/// Method to register the publisher into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterAsyncRosPublisher(BT::BehaviorTreeFactory& factory,
                     const std::string& registration_ID,
                     ros::NodeHandle& node_handle)
{
  NodeBuilder builder = [&node_handle](const std::string& name, const NodeConfig& config) {
    return std::make_unique<DerivedT>(node_handle, name, config );
  };

  TreeNodeManifest manifest;
  manifest.type = getType<DerivedT>();
  manifest.ports = DerivedT::providedPorts();
  manifest.registration_ID = registration_ID;
  const auto& basic_ports = AsyncRosPublisherNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_ASYNC_PUBLISHER_NODE_HPP_