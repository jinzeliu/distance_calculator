#include "driver.h"
#include "utils/utils.h"
#include "utils/ros_utils.h"

namespace bipedlab
{

Driver::Driver(ros::NodeHandle& nh):
    data_received_(false), pose_updated_(false)
{
    nh_ = nh;
    if (!Driver::getParameters_())
    {
        debugger::debugTitleTextOutput("[Driver]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[Driver] Not enough parameters: ",
                                   "Using default values", 10, BR, BOLD);
        debugger::debugTitleTextOutput("[Driver]", "", 10, BR, BOLD);
        utils::pressEnterToContinue();
    }
    else
    {
        debugger::debugColorOutput("[Driver] Received all parameters", "", 10, BC);
    }

    // subscribers
    pose_sub_ = nh_.subscribe(std::string(robot_pose_topic_), 1,
                               &Driver::getPoseCallBack_, this);


    boost::thread ros_spin(&Driver::spin_, this);

    // wait for data arriving
    Driver::waitForData_();

    boost::thread print_result(&Driver::print_result_, this);

    while (1)
    {
        sleep(0.2);
    }
}

void Driver::waitForData_()
{
    while (ros::ok() && !pose_updated_)
    {
      ROS_WARN_THROTTLE(1, "Received all poses: %i", pose_updated_);
      sleep(0.5);
    }
    ROS_INFO("Received all poses: %i", pose_updated_);
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParameters] ");
    bool received_all = true;

    // log command
    ros_utils::checkROSParam(nh_, "robot_pose_topic", robot_pose_topic_,
            getNameOf(robot_pose_topic_), title_name, received_all);

    if (!received_all)
    {
        robot_pose_topic_ = std::string("/cassie/pose");
    }

    return received_all;
}

void Driver::spin_()
{
    while (ros::ok()){
        ros::spinOnce();
    }
}

void Driver::getPoseCallBack_(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
    //debugger::debugColorTextOutput("In PoseCallBack()",3);
    pose_lock_.lock();
    ros::Time t = pose.header.stamp;
    double x = pose.pose.pose.position.x, y = pose.pose.pose.position.y, z = pose.pose.pose.position.z;
    if (!pose_updated_)
    {
        pose_updated_ = true;
        current_x_ = x;
        current_y_ = y;
        current_z_ = z;
        trajectory_2d_ = 0.0;
        trajectory_3d_ = 0.0;
        start_time_ = t;
    }
    else
    {
        trajectory_2d_ += sqrt((x - current_x_)*(x - current_x_) + (y - current_y_)*(y - current_y_));
        trajectory_3d_ += sqrt((x - current_x_)*(x - current_x_) + (y - current_y_)*(y - current_y_) + (z - current_z_)*(z - current_z_));
        current_x_ = x;
        current_y_ = y;
        current_z_ = z;
        current_time_ = t;
    }
    pose_lock_.unlock();
}

void Driver::print_result_()
{
    while (ros::ok()){
        sleep(2);
        pose_lock_.lock();
        std::cout << "================================" << std::endl;
        std::cout << "total time(sec): " << current_time_.sec - start_time_.sec << std::endl;
        //std::cout << "start time(nsec): " << start_time_.nsec << std::endl;
        //std::cout << "current time(nsec): " << current_time_.nsec << std::endl;
        std::cout << "2d trajectory length: " << trajectory_2d_ << std::endl;
        std::cout << "3d trajectory length: " << trajectory_3d_ << std::endl;
        pose_lock_.unlock();
    }
}

Driver::~Driver() { }

}
