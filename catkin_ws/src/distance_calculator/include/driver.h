#ifndef DRIVER_H
#define DRIVER_H

#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace bipedlab
{

class Driver
{
private:

    //initialization functions
    void waitForData_();
    bool getParameters_();
    void spin_();

    //callback functions
    //void getClickPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg);
    void getPoseCallBack_(const geometry_msgs::PoseWithCovarianceStamped& pose);

    void print_result_();

    // ROS
    ros::NodeHandle nh_;

    //subscriber
    ros::Subscriber pose_sub_;

    // subscribe channel
    std::string robot_pose_topic_;

    // if data received
    bool data_received_;

    // mutex locks
    boost::mutex pose_lock_;

    // if data updated
    bool pose_updated_;

    //pose and dist data
    double current_x_, current_y_, current_z_, trajectory_2d_, trajectory_3d_;
    ros::Time start_time_, current_time_;

public:

    Driver(ros::NodeHandle& nh);
    virtual ~Driver();

};

}

#endif
