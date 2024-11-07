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

## How to create a condition node that communicates with rostopic
ROS condition node wrapper: https://github.com/NTU-Mecatron/BehaviorTree.ROS/blob/BT-JH/include/behaviortree_ros/bt_condition_node.h

Example usage of wrapper: https://github.com/NTU-Mecatron/BehaviorTree.ROS/blob/BT-JH/src/condition_nodes.h

[Screencast from 02-09-24 23:01:26.webm](https://github.com/user-attachments/assets/96a6ad92-5b24-453c-bfec-ff504ef33087)

## How to launch BT for navigating through the gate

1. (Setup) Run essential packages: ros_tcp_endpoint, yolo_object_detection and common_action_service_servers

    Open simulation and launch `ros_tcp_endpoint` as usual.

    If you have not cloned yolo_object_detection and common_action_service_servers, paste these command lines on terminal:

    ```bash
    git clone https://github.com/NTU-Mecatron/yolo_object_detection.git
    git clone https://github.com/NTU-Mecatron/common_action_service_servers.git
    ```

    After cloning, remember to switch to branch `bao` in both packages:

    ```bash
    cd yolo_object_detection
    git checkout bao
    cd ../common_action_service_servers
    git checkout bao
    cd ..
    ```

    Then, build the workspace and launch the packages (remember to run roscore):

    ```bash
    catkin_make

    // On the first terminal
    source devel/setup.bash
    roslaunch yolo_object_detection launch.launch

    // On the second terminal
    source devel/setup.bash
    roslaunch common_action_service_servers launch_action.launch 
    ```
        
    After running, we expect that the yolo model does not run because I set that it only starts when we call to it via a service server. There are two ways to start the yolo model, first is to run the BT with the node OnYolo (will show later), and second is to run this command line on another terminal (just do for debugging):

    ```bash
    source devel/setup.bash
    rosservice call /on_yolo "turn_on: true"
    ```


2. Run essential UI software: Rviz and Groot2

    Start Rviz and read the frame from yolo_object_detecion as usual:

    ```bash
    rviz
    ```

    Then open the Groot2, press "Connect". Nothing happens now since we have not launched the packages.


3. Setup and Launch the BT

    Ensure you started the simulation and ros_tcp_endpoint, as well as other essential packages in step 1. 

    To clone the BT packages, please clone and switch to branch `bao`:

    ```bash
    git clone https://github.com/NTU-Mecatron/BehaviorTree.ROS.git
    git checkout bao
    ```

    Launch the packages:

    ```bash
    source devel/setup.bash
    roslaunch behaviortree_ros nav_through_gate.launch
    ```

    Check the Rviz and Groot2 now. Enjoy!
    
