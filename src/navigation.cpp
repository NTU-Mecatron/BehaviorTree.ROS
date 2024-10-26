#include <ros/ros.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>

#include "on_yolo_client.h"
#include "center_client.h"
#include "search_rotate.h"
#include "print_value.h"
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

    factory.registerNodeType<PrintValue>("PrintValue");

    RegisterRosService<OnYoloClient>(factory, "OnYolo", nh);
    RegisterRosAction<CenterClient>(factory, "Center", nh);
    RegisterRosAction<SearchRotateClient>(factory, "RotateSearch", nh);

     
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
        tree.sleep(std::chrono::milliseconds(100));
    }
    std::cout << status << std::endl;
    return 0;
}