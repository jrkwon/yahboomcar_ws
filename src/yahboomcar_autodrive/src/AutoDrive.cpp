#include "yahboomcar_autodrive/AutoDrive.h"

/**
 * @brief Construct a new Auto Drive:: Auto Drive object
 * 
 * @param nh ros::NodeHandle
 */
AutoDrive::AutoDrive(ros::NodeHandle nh) :
    nh_(nh),
    pose_map_x_(0.408796),
    pose_map_y_(1.876130)
{
    car_twist_ = geometry_msgs::Twist();
    MoveBaseStatus_ = actionlib_msgs::GoalStatusArray();
    traffic_flag_ = 0;
    control_rate_1_ = 50.0;
    control_rate_2_ = 50.0;
    pose_map_q_ = tf::Quaternion();

    car_vel_pub_=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    car_vel_sub_=nh.subscribe<geometry_msgs::Twist>("/nav_cmd_vel",1, &AutoDrive::NavCmdVelCallback, this);
    move_base_status_sub_=nh.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status",1,&AutoDrive::MoveBaseStatusCallback,this);
    move_base_simple_goal_pub_=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    sign_sub_ = nh.subscribe<std_msgs::Int32>("/traffic",1,&AutoDrive::SignCallback,this);
    buzzer_pub_ = nh.advertise<std_msgs::Bool>("Buzzer",1);




    pac_ = new MoveBaseClient("move_base",false);
}

/**
 * @brief thread 2 : Processing speed information based on recognition results
 * 
 */
void AutoDrive::Start()
{
    ROS_INFO("in thread Start");
    ros::Rate rate(control_rate_2_);
    while(ros::ok())
    {
        //TODO : Processing speed information based on recognition results
        // such as : if we detect low speed mode , we should set linear velocity x as 0.8 time or less.
        // if we detect parking mode , we should send some point
        //SendNavGoal(pose,PUBLISHER);

        double x,y,q[4];
        GetPoseMap(x,y,q); // get pose now


        ROS_INFO("get transform is %f %f",x,y); // for debug
        

        //processing vel info , for 3 , 4 , 5 , 6 , 8
        //for other sign , we should set point for navigation

        //TODO: set point for navigation
        if(traffic_flag_ == 0)
        {
            //we haven't detect any traffic sign
        }
        else if(traffic_flag_ == 3)
        {
            //Beep  ,  We should call the buzzle
            std_msgs::Bool buzzer;
            buzzer.data = true;
            buzzer_pub_.publish(buzzer);
        }
        else if(traffic_flag_ == 4)
        {
            //sidewalk , we should slow down
            car_twist_.linear.x = car_twist_.linear.x * 0.5;
        }
        else if(traffic_flag_==5)
        {
            //Stop
            car_twist_ = geometry_msgs::Twist();
        }
        else if(traffic_flag_==6)
        {
            //Slow down
            car_twist_.linear.x = car_twist_.linear.x * 0.5;
        }
        else if(traffic_flag_ == 8)
        {
            //School , we should slow down
            car_twist_.linear.x = car_twist_.linear.x*0.5;
        }

        car_vel_pub_.publish(car_twist_);   // for the car motion
        ros::spinOnce();
        rate.sleep();
    }
}


/**
 * @brief thread 1 : tf listener
 * 
 */
void AutoDrive::tfListener()
{
    ROS_INFO("in thread tfListener");
    ros::Rate rate(control_rate_1_);
    while(ros::ok())
    {
        tf::StampedTransform transform;
        // if(tf_listener_.canTransform("base_footprint","map",ros::Time::now()))
        // {
        //     ROS_ERROR("can not find tf");
        //     rate.sleep();
        //     continue;
        // }
        try
        {
            ros::Time now=ros::Time::now();
            //parent frame : map , child frame : base_footprint
            tf_listener_.waitForTransform("base_footprint","map",now,ros::Duration(0.1));
            tf_listener_.lookupTransform("/base_footprint", "/map",now, transform); 
            double x,y;
            tf::Vector3 translation = transform.getOrigin();
            x=translation.getX();
            y=translation.getY();
            tf::Quaternion quaternion = transform.getRotation();
            SetPoseMap(x,y,quaternion);
            //ROS_INFO("set transform is %f %f",quaternion.getW(),quaternion.getZ()); // for debug
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.1).sleep();
        }
        rate.sleep();
    }
}


/** 
 * @brief nav_cmd_vel callback function
 * 
 * @param twist_msgs 
 */
void AutoDrive::NavCmdVelCallback(const geometry_msgs::Twist twist_msgs)
{
    //std::cout<<"in callback"<<std::endl;
    //geometry_msgs::Twist twist(twist_msgs);//copy
    car_twist_.angular.x=twist_msgs.angular.x;
    car_twist_.angular.y=twist_msgs.angular.y;
    car_twist_.angular.z=twist_msgs.angular.z;
    car_twist_.linear.x=twist_msgs.linear.x;
    car_twist_.linear.y=twist_msgs.linear.y;
    car_twist_.linear.z=twist_msgs.linear.z;
}

/**
 * @brief move_base/status callback function
 * 
 * @param status 
 */
void AutoDrive::MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray status)
{
    //https://blog.csdn.net/Draonly/article/details/103292502
    // //目标点尚未被处理
    // uint8 PENDING=0		
    // //目标点正在被处理		
    // uint8 ACTIVE=1		
    // //目标在开始执行后收到了取消请求，并已完成其执行			
    // uint8 PREEMPTED=2
    // //目标被成功实现
    // uint8 SUCCEEDED=3  
    // //目标在执行期间被中止，由于一些故障（失败）
    // uint8 ABORTED=4		
    // //目标尚未被处理就被拒绝，由于目标不可达或无效
    // uint8 REJECTED=5		
    // //在目标开始执行后，收到取消请求，尚未执行完成
    // uint8 PREEMPTING=6  
    //  //在目标开始执行前收到了取消请求，但action server尚未确认
    // uint8 RECALLING=7  
    // //在目标开始执行前收到了取消请求，且被成功取消
    // uint8 RECALLED=8   
    //  //action 客户端可以确认目标已经丢失，action服务端不应该再通过网络发送。
    // uint8 LOST=9  

    MoveBaseStatus_.header.frame_id = status.header.frame_id;
    MoveBaseStatus_.header.stamp = status.header.stamp;
    MoveBaseStatus_.status_list=status.status_list;    
    //std::cout<<int(status.status_list[0].status)<<std::endl;
}

/**
 * @brief Traffic sign callback function
 * 
 * @param sign_msgs 
 */
void AutoDrive::SignCallback(const std_msgs::Int32 sign_msgs)
{
    traffic_flag_ = sign_msgs.data;
}


/**
 * @brief We should give pose ,so that this function will send navigation goal to move_base
 * @param  pose Where we desire to go. 
 * @param  sendType  choose send Type : 1. by Client 2. by Publisher
 */
bool AutoDrive::SendNavGoal(geometry_msgs::PoseStamped pose,int sendType)
{
    if(sendType == CLIENT)
    {
        move_base_msgs::MoveBaseGoal goal;

        //wait for the action server to come up (5s)
        while(!pac_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
            return false;
        }

        //we'll send a goal to the robot to move 1 meter forward
        //we can change the frame id to map , if we send the global goal
        //in this project , we often send global goal
        goal.target_pose.header.frame_id = pose.header.frame_id;
        goal.target_pose.header.stamp = pose.header.stamp;

        //pose descripe by quaternion
        goal.target_pose.pose.position.x = pose.pose.position.x;
        goal.target_pose.pose.position.y = pose.pose.position.y;
        goal.target_pose.pose.position.z = pose.pose.position.z;
        goal.target_pose.pose.orientation.w = pose.pose.orientation.w;
        goal.target_pose.pose.orientation.x = pose.pose.orientation.x;
        goal.target_pose.pose.orientation.y = pose.pose.orientation.y;
        goal.target_pose.pose.orientation.z = pose.pose.orientation.z;

        //call the client
        ROS_INFO("Sending goal");
        pac_->sendGoal(goal);

        pac_->waitForResult();

        if(pac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Success");
            return true;
        }
        else
        {
            ROS_INFO("Fail");
            return false;
        }        
    }
    else if(sendType == PUBLISHER)
    {
        if(pose.header.frame_id != "map")
        {
            ROS_INFO("Fail , by publisher , the frame id must be map");
            return false;
        }
        move_base_simple_goal_pub_.publish(pose);
        return true;
    }
}

/**
 * @brief  We use this function to get car pose in map
 * 
 * @param x pose x
 * @param y pose y
 * @param q tf::Quaternion , quaternion which can show the rotate information
 */
void AutoDrive::GetPoseMap(double &x , double &y , double q[])
{
    std::unique_lock<std::mutex> lock(tfmutex);
    x = pose_map_x_;
    y = pose_map_y_;
    q[0] = pose_map_q_w_;
    q[1] = pose_map_q_x_;
    q[2] = pose_map_q_y_;
    q[3] = pose_map_q_z_;
    //ROS_INFO("set transform is %f %f",pose_map_x_,pose_map_y_);
}

void AutoDrive::SetPoseMap(double x , double y , tf::Quaternion &q)
{
    std::unique_lock<std::mutex> lock(tfmutex);
    pose_map_x_ = x;
    pose_map_y_ = y;
    pose_map_q_w_ = q.getW();
    pose_map_q_x_ = q.getX();
    pose_map_q_y_ = q.getY();
    pose_map_q_z_ = q.getZ();
    //for debug
    //ROS_INFO("Saved transform is %f %f",pose_map_x_,pose_map_y_);
}


