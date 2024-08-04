# Tutorial 1: Installation and introduction to overall workflow for BT-ROS

**Table of Contents**
 
- [Installation](#installation)
- [Overall workflow for BT-ROS integration](#overall-workflow-for-bt-ros-integration)
- [BT action node vs ROS action node](#bt-action-node-vs-ros-action-node)
- [ROS topics vs services vs actions](#ros-topics-vs-services-vs-actions)

## Installation

```bash
cd /path/to/catkin_ws/src

git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git

git clone https://github.com/NTU-Mecatron/BehaviorTree.ROS.git

cd ..
catkin_make
```

> Tips: to avoid the long logging message of catkin_make due to the many examples in `BehaviorTree.CPP`, you can fully comment out the CMakelists.txt in the `examples`, `sample_nodes` and `tests` subdirectory of `BehaviorTree.CPP`.

## Overall workflow for BT-ROS integration

> Warning: before reading this tutorial, make sure you have a basic understanding of Behavior Trees. Please visit the [official documentation](https://www.behaviortree.dev/) and finish the following sections: `About`, `Basic concepts` (can skip `The XML Schema`) and read the `Tutorials (Basic)` up until and including tutorial 4.

Each `action node` in a Behavior Tree should be either a `ROS service client node` or `ROS action client node` that performs a specific task. For example, a node that calls a ROS service to open/close a gripper, or a node that calls an action server to "move from A to B". The servers for these services/actions can be implemented in any language (C++, Python, etc.) in existing packages in your own project.

The `BehaviorTree.ROS` package provides two types of nodes: `RosServiceNode` and `RosActionNode`. These nodes are wrappers around the `ros::ServiceClient` and `actionlib::SimpleActionClient` classes, respectively. When writing your own mission planner, it is recommended that you use these wrappers to call ROS services and actions.

### BT action node vs ROS action node

There is common confusion around these two concepts which could complicate your BT-ROS workflow. `BT action node` is a node in the BT that performs a specific task and can be done with either ROS service or ROS action. However, ROS action is a specific type of communication in ROS and should comprise of two nodes: `action server` and `action client`. `action client` will send a goal to the `action server` and the server will respond with a result after some long running time.

As such, when you are writing a BT node that calls a ROS action, you should have two separate nodes: `action server` and `action client`. The `action client` will be a `BT action node` in the BT, while the `action server` will be a separate node that you can implement in any language.

### ROS topics vs services vs actions

The original answer has been posted on [ROS docs](https://docs.ros.org/en/foxy/How-To-Guides/Topics-Services-Actions.html). Here is a summary:

- **Topics**: A topic is a one-directional, many-to-many communication channel. Used for sending consistent data streams, such as sensor data or positional data.

- **Services**: A service is a two-way, one server to many clients communication channel. Used for sending really short, blocking requests for something to be done. For example, call a service to open/close gripper or drop a ball. There should not be any delay between sending the request and receiving the response.

- **Actions**: An action is a two-way, one server to many clients communication channel. Used for sending long running and asynchronous requests. For example, call an action to move from A to B. The advantage of actions is that they can provide feedback during the execution of the goal, and while the action is being executed, the client can cancel the goal. It also does not block the main thread.

Linking back to BT concepts:

- A service client node is implemented as a wrapper around `SyncActionNode` in BT because it is short and blocking.

- An action client node is implemented as a wrapper around `StatefulActionNode` (the rephrased term for `AsyncActionNode` in v4) in BT because it is long running and asynchronous.

However, you will notice in the [ROS Action wrapper](include/behaviortree_ros/bt_action_node.h) that the `RosActionNode` is implemented as a wrapper around `ActionNodeBase` in BT, which is the base class for almost all types of action node in BT. The likely reason is because for `StatefulActionNode`, you need to implement the `onRunning` method, which is not necessary in BT-ROS integration.