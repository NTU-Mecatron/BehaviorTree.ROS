#include "../src/on_yolo_client.h"
#include "../src/print_value.h"
#include "../src/call_gripper_client.h"
#include "../src/call_camera_Xangle.h"
#include "../src/center_client.h"
#include "../src/search_bin.h"
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
    RegisterRosService<OnYoloClient>(factory, "OnYolo", nh);
    RegisterRosService<CameraXAngleClient>(factory, "CameraXAngle", nh);
    RegisterRosAction<SearchBinClient>(factory, "SearchBin", nh);
    RegisterRosAction<CenterClient>(factory, "Center", nh);
    RegisterRosAction<CenterClient>(factory, "MoveDown", nh);
    RegisterRosAction<CenterClient>(factory, "MoveUp", nh);
    RegisterRosService<GripperClient>(factory, "Gripper", nh);

    // RegisterRosAction<CenterMoveUpClient>(factory, "CenterMoveUp", nh);
    // RegisterRosAction<CenterMoveDownClient>(factory, "CenterMoveDown", nh);
    // RegisterRosAction<DoTaskClient>(factory, "DoTask", nh);
    // RegisterRosService<AddTwoIntsClient>(factory, "AddTwoInts", nh);
    // RegisterRosAction<FibonacciClient>(factory, "Fibonacci", nh);
    // End of section

    // Modify your xml_file_path here
    std::string xml_file_path;
    ros::param::get("~tree_node_model_xmlfile_path", xml_file_path);

    std::ofstream out(xml_file_path);
    out << BT::writeTreeNodesModelXML(factory);
    out.close();

    return 0;
}