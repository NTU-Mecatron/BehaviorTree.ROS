# BehaviorTree.ROS

This is a fork of the original [BehaviorTree.ROS](https://github.com/BehaviorTree/BehaviorTree.ROS/tree/master) repo, with more elaborate instructions on how to incorporate Behavior Trees with ROS1. The scripts have also been modified to work with the latest version of BehaviorTree.CPP (v4).

## Overall information

Currently, two wrappers are provided:

- [RosServiceNode](include/behaviortree_ros/bt_service_node.h), which can be used to call
  [ROS Services](http://wiki.ros.org/Services)

- [RosActionNode](include/behaviortree_ros/bt_action_node.h) that, similarly, is a wrapper around
  [actionlib::SimpleActionClient](http://wiki.ros.org/actionlib).

This package also provides an example of how to implement BT in ROS. Specifically:

- `src/main.cpp` is an example of how to create a simple BT that calls a ROS Service and a ROS Action, and connect it to Groot2.

- In the `src` directory, you can also find `add_two_ints_client.h`, `fibonacci_client.h` and `print_value.h` which are header files to define the three different BT action nodes used in the example.

## Tutorials

- [**Tutorial 1: Installation and introduction to overall workflow for BT-ROS**](docs/tutorial1.md)

- [**Tutorial 2: Writing a simple BT node that calls a ROS Service**](docs/tutorial2.md)

- [**Tutorial 3: Writing a simple BT node that calls a ROS Action**](docs/tutorial3.md)

- [**Tutorial 4: Writing BT, visualisation and logging**](docs/tutorial4.md)

To skip to running the example, go to [Tutorial 4](docs/tutorial4.md#running-the-entire-example).

## How to change your current nodes for Task into a service server
Refer to this https://github.com/NTU-Mecatron/BehaviorTree.ROS/blob/BT-JH/src/python_service.py
