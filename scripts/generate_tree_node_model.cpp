#include "../src/action_yolo.h"
#include "../src/action_center.h"
#include "../src/action_blind_movement.h"
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
    // factory.registerNodeType<PrintValue>("PrintValue");
    RegisterRosAction<YoloClient>(factory, "OnYolo", nh);
    RegisterRosAction<CenterClient>(factory, "Center", nh);
    RegisterRosAction<BlindMovementClient>(factory, "BlindMovement", nh);
    
    // End of section

    // Modify your xml_file_path here
    std::string xml_file_path;
    ros::param::get("~tree_node_model_xmlfile_path", xml_file_path);

    std::ofstream out(xml_file_path);
    out << BT::writeTreeNodesModelXML(factory);
    out.close();

    return 0;
}