#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <behaviortree_cpp/bt_factory.h>
#include <array>
#include <sstream>

namespace BT
{
    template <> inline std::array<float, 3> convertFromString(StringView str)
    {
        std::array<float, 3> result;
        std::stringstream ss(str.data());
        std::string item;
        size_t index = 0;
        while (std::getline(ss, item, ',') && index < 3)
        {
            result[index++] = static_cast<float>(std::stof(item));
        }
        if (index != 3)
        {
            throw BT::RuntimeError("Invalid input for std::array<float, 3>");
        }
        return result;
    }
}

#endif // CONVERSIONS_H