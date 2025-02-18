#include "add_two_ints_client.h"
#include "fibonacci_client.h"
#include "print_value.h"
#include "check_bool.h"
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
    factory.registerNodeType<PrintValue>("PrintValue");
    RegisterRosSubscriber<CheckBool>(factory, "CheckBool", nh);
    RegisterRosService<AddTwoIntsClient>(factory, "AddTwoInts", nh);
    RegisterRosAction<FibonacciClient>(factory, "Fibonacci", nh);
    // End of section

    // Modify your xml_file_path here
    std::string xml_file_path;
    ros::param::get("~tree_node_model_xmlfile_path", xml_file_path);

    std::ofstream out(xml_file_path);
    out << BT::writeTreeNodesModelXML(factory);
    out.close();

    return 0;
}