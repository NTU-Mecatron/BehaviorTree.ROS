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

#ifndef BEHAVIOR_TREE_BT_PUBLISHER_NODE_HPP_
#define BEHAVIOR_TREE_BT_PUBLISHER_NODE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <ros/ros.h>

namespace BT
{

/**
 * Base Action to implement a ROS Publisher
 */

template<class MessageT>
class RosPublisherNode : public BT::SyncActionNode
{
protected:

  RosPublisherNode(ros::NodeHandle& nh, const std::string& name, const BT::NodeConfig& conf):
  BT::SyncActionNode(name, conf), node_(nh) 
  {
    const std::string topic = getInput<std::string>("topic_name").value();
    const unsigned queue_size = getInput<unsigned>("queue_size").value();
    publisher_ = node_.advertise<MessageT>(topic, queue_size);
  }

public:

  using BaseClass    = RosPublisherNode<MessageT>;
  using MessageType  = MessageT;

  RosPublisherNode() = delete;

  virtual ~RosPublisherNode() = default;

  /// These ports will be added automatically if this Node is
  /// registered using RegisterRosPublisher<DeriveClass>()
  static PortsList providedPorts()
  {
    return  {
      InputPort<std::string>("topic_name", "name of the ROS topic"),
      InputPort<unsigned>("queue_size", 1, "queue size"),
      };
  }

  /// User must implement this method.
  virtual bool setMessage(MessageT& msg) = 0;

protected:

  ros::Publisher publisher_;
  ros::NodeHandle& node_;

  NodeStatus tick() override
  {
    MessageT msg_;
    if( setMessage(msg_) )
    {
      publisher_.publish(msg_);
      return NodeStatus::SUCCESS;
    }
    else
    {
      return NodeStatus::FAILURE;
    }
  }
};

/// Method to register the publisher into a factory.
/// It gives you the opportunity to set the ros::NodeHandle.
template <class DerivedT> static
  void RegisterRosPublisher(BT::BehaviorTreeFactory& factory,
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
  const auto& basic_ports = RosPublisherNode< typename DerivedT::MessageType>::providedPorts();
  manifest.ports.insert( basic_ports.begin(), basic_ports.end() );

  factory.registerBuilder( manifest, builder );
}


}  // namespace BT

#endif  // BEHAVIOR_TREE_BT_PUBLISHER_NODE_HPP_