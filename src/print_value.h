#ifndef PRINT_VALUE_H
#define PRINT_VALUE_H

using namespace BT;

class PrintValue : public SyncActionNode
{
public:

    PrintValue(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config) {}

    // Static method to provide the list of ports
    static PortsList providedPorts() {
        return{ InputPort<int>("message") };
    }

    // Here we define the tick function inside the header file because it is a simple function
    NodeStatus tick() override {
        int value = 0;
        if (getInput("message", value)) {
            std::cout << "PrintValue: " << value << std::endl;
            return NodeStatus::SUCCESS;
        }
        else {
            std::cout << "PrintValue FAILED " << std::endl;
            return NodeStatus::FAILURE;
        }
    };

};

#endif