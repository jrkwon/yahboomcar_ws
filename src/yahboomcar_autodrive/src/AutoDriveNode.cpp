#include <ros/ros.h>
#include "yahboomcar_autodrive/AutoDrive.h"

#include<thread>
#include<unistd.h>
#include<mutex>


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "yahboom_autodrive");
    ros::NodeHandle nh;
    //ros::NodeHandle nh_private("~");
    AutoDrive autodrive(nh);
    //ros::spin();

    //move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped pose;

    //we'll send a goal to the robot to move 1 meter forward
    //we can change the frame id to map , if we send the global goal
    //in this project , we often send global goal
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();

    //pose descripe by quaternion
    pose.pose.position.x = 0.55;
    pose.pose.position.y = 2.95;
    pose.pose.orientation.w = 1.0;

    autodrive.SendNavGoal(pose,AutoDrive::PUBLISHER);

    std::thread threadstart(&AutoDrive::Start , &autodrive);
    std::thread threadtf(&AutoDrive::tfListener, &autodrive);
    threadstart.join();
    threadtf.join();

    //ros::spin();

    return 0;
}
