<<<<<<< HEAD:examples/main.cpp
#include "add_two_ints_client.h"
#include "fibonacci_client.h"
#include "check_bool.h"
#include "send_bool.h"
#include "print_value.h"
=======
>>>>>>> a14b303c1bc28cdd0f42cec10705e1bcdffbc926:src/nav_through_gate.cpp
#include <ros/ros.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

#include "action_center.h"
#include "action_blind_movement.h"
#include "action_yolo.h"

// headers for signal handling
#include <unistd.h>
#include <stdio.h>
#include <signal.h>

using namespace BT;

volatile sig_atomic_t stop;
void inthand(int signum) {
    stop = 0;
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "test_behavior_tree");
    ros::NodeHandle nh;

    BehaviorTreeFactory factory;

<<<<<<< HEAD:examples/main.cpp
    factory.registerNodeType<PrintValue>("PrintValue");
    RegisterRosSubscriber<CheckBool>(factory, "CheckBool", nh);
    RegisterRosPublisher<SendBool>(factory, "SendBool", nh);
    RegisterRosService<AddTwoIntsClient>(factory, "AddTwoInts", nh);
    RegisterRosAction<FibonacciClient>(factory, "Fibonacci", nh);
    
=======
    RegisterRosAction<YoloClient>(factory, "Yolo", nh);
    RegisterRosAction<CenterClient>(factory, "Center", nh);
    RegisterRosAction<BlindMovementClient>(factory, "BlindMovement", nh);

     
>>>>>>> a14b303c1bc28cdd0f42cec10705e1bcdffbc926:src/nav_through_gate.cpp
    std::string xml_file;
    ros::param::get("~xml_file", xml_file);
    auto tree = factory.createTreeFromFile(xml_file);

    std::string log_file;
    ros::param::get("~log_file", log_file);
    FileLogger2 logger(tree, log_file);

    // Connect the Groot2Publisher. This will allow Groot2 to get the tree and poll status updates.
    const unsigned port = 1667;
    Groot2Publisher publisher(tree, port);

    NodeStatus status = tree.tickOnce();

    signal(SIGINT, inthand);

    while( !stop && ros::ok() && (status == NodeStatus::RUNNING))
    {
        ros::spinOnce();
        status = tree.tickOnce();
        tree.sleep(std::chrono::milliseconds(20));
    }
    std::cout << status << std::endl;
    return 0;
}