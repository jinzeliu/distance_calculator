#include <string>
#include <ros/node_handle.h>
#include "utils/debugger.h"
#include "utils/nameof.h"


namespace bipedlab
{
namespace ros_utils
{
    // ros_utils::checkROSParam(nh_, "map_frame", map_frame_,
    //         getNameOf(map_frame_), received_all);
    template <class T>
    void checkROSParam(const ros::NodeHandle& nh,
                       const std::string& ros_param_name,
                       T& member_name, std::string variable_name, 
                       const std::string& title_name,
                       bool& is_received)
    {
        if (!nh.getParam(ros_param_name, member_name))
        {
            debugger::debugColorOutput(title_name,
                    "missing " + variable_name, 10, Y, N);
            is_received = false;
        }
    }
        
} /* ros_utils */ 
    
} /* bipedlab */ 

