<<<<<<< HEAD:examples/generate_tree_node_model.cpp
#include "add_two_ints_client.h"
#include "fibonacci_client.h"
#include "print_value.h"
#include "check_bool.h"
=======
#include "../src/action_yolo.h"
#include "../src/action_center.h"
#include "../src/action_blind_movement.h"
>>>>>>> a14b303c1bc28cdd0f42cec10705e1bcdffbc926:scripts/generate_tree_node_model.cpp
#include <ros/ros.h>
#include <behaviortree_cpp/xml_parsing.h>
using namespace BT;

// headers for writing and file operation
#include <iostream>
#include <fstream>

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "generate_tree_node_model");
    ros::NodeHandle nh;

    BehaviorTreeFactory factory;

    // ----------------- Register your custom nodes ----------------------
    // You need to edit this section to include your custom nodes
<<<<<<< HEAD:examples/generate_tree_node_model.cpp
    factory.registerNodeType<PrintValue>("PrintValue");
    RegisterRosSubscriber<CheckBool>(factory, "CheckBool", nh);
    RegisterRosService<AddTwoIntsClient>(factory, "AddTwoInts", nh);
    RegisterRosAction<FibonacciClient>(factory, "Fibonacci", nh);
=======
    // factory.registerNodeType<PrintValue>("PrintValue");
    RegisterRosAction<YoloClient>(factory, "OnYolo", nh);
    RegisterRosAction<CenterClient>(factory, "Center", nh);
    RegisterRosAction<BlindMovementClient>(factory, "BlindMovement", nh);
    
>>>>>>> a14b303c1bc28cdd0f42cec10705e1bcdffbc926:scripts/generate_tree_node_model.cpp
    // End of section

    // Modify your xml_file_path here
    std::string xml_file_path;
    ros::param::get("~tree_node_model_xmlfile_path", xml_file_path);

    std::ofstream out(xml_file_path);
    out << BT::writeTreeNodesModelXML(factory);
    out.close();

    return 0;
}