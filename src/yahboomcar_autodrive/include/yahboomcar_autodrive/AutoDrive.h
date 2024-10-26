#ifndef AUTODRIVE_H
#define AUTODRIVE_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include<thread>
#include<unistd.h>
#include<mutex>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AutoDrive
{
    public:
    //for ROS
        ros::NodeHandle nh_;
        ros::Publisher car_vel_pub_;
        ros::Subscriber car_vel_sub_;//from navigation
        ros::Subscriber move_base_status_sub_;
        ros::Subscriber sign_sub_;
        ros::Publisher move_base_simple_goal_pub_;
        ros::Publisher buzzer_pub_;// buzzer

        MoveBaseClient *pac_;// for sending goal points

        tf::TransformListener tf_listener_;

        //for thread
        std::mutex tfmutex;

    public:
        //construction functions
        AutoDrive(){;};
        ~AutoDrive(){;};
        AutoDrive(ros::NodeHandle nh);

    public:
    //for function
        //callback function
        void NavCmdVelCallback(const geometry_msgs::Twist twist_msgs);
        void MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray status);
        void SignCallback(const std_msgs::Int32 sign_msgs);

        // send goal point
        enum SendType
        {
            CLIENT=0,
            PUBLISHER=1
        };
        bool SendNavGoal(geometry_msgs::PoseStamped pose,int sendType);

        //get car pose in map
        void GetPoseMap(double &x , double &y , double q[]);
        void SetPoseMap(double x,double y, tf::Quaternion &q);

        //get car pose in odom
        void GetPoseOdom(double &x,double &y,double &yaw);
        void SetPoseOdom(double x,double y,double yaw);

        //main function
        void Start();

    public:
    //for odom
        //if we use tf listner , we probably have to use multi thread
        double pose_map_x_;
        double pose_map_y_;
        double pose_map_q_x_;
        double pose_map_q_y_;
        double pose_map_q_z_;
        double pose_map_q_w_;
        tf::Quaternion pose_map_q_;
        //for cmd_vel
        geometry_msgs::Twist car_twist_;

        //for tf
        void tfListener();

        //move base status
        actionlib_msgs::GoalStatusArray MoveBaseStatus_;

        //for traffic sign
        int traffic_flag_;

        //for navigation point
        std::vector<geometry_msgs::PoseStamped> nav_pose_vector_; // the point set

    public:
    //for param
    //such as : control rate , some topic name and so on
        double control_rate_1_; // thread 1 (tf) rate
        double control_rate_2_; // thread 2 (start) rate

        //For convenience, we set the topic name as param , (not finished)
        std::string frame_id_;
        std::string vel_cmd_topic_;


};




#endif