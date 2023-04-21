/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 * 
 * from https://docs.px4.io/master/en/ros/mavros_offboard.html
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>

mavros_msgs::State current_state;
mavros_msgs::PositionTarget pose_vel;
geometry_msgs::PoseStamped g_pose;
geometry_msgs::PoseStamped drone_pose_feedback;
geometry_msgs::PoseStamped tag_pose_body;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_pose = *msg;
    // ROS_INFO("I heard pose: x=%f, y=%f, z=%f", 
    //          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}
void lpp_callback(const geometry_msgs::PoseStamped::ConstPtr& lpp_msg){

drone_pose_feedback = *lpp_msg;

}
void body_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    tag_pose_body = *msg;
}

int main(int argc, char **argv)
{
    //Define the node name
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //Define the Target pose pubisher    
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    //Define the tag inertial frame subscriber
    ros::Subscriber sub_pose_artag = nh.subscribe("/tag_detections/tagpose_inertial",1000,callback);
    //Define the drone pose subscriber
    ros::Subscriber local_info_sub = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, lpp_callback);
    //Define the tag's pose in body frame
    ros::Subscriber artag_body_pose = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body",1000, body_callback);
    ros::Rate rate(20.0);

    //Define the drone pose message identifier
    mavros_msgs::PositionTarget drone_pose_cmd;
    drone_pose_cmd.position.x = 0;
    drone_pose_cmd.position.y = 0;
    drone_pose_cmd.position.z = 1;
    drone_pose_cmd.type_mask = drone_pose_cmd.IGNORE_VX | drone_pose_cmd.IGNORE_VY | drone_pose_cmd.IGNORE_VZ | drone_pose_cmd.IGNORE_AFZ | drone_pose_cmd.IGNORE_AFY | drone_pose_cmd.IGNORE_AFX;
    drone_pose_cmd.coordinate_frame = drone_pose_cmd.FRAME_LOCAL_NED;
    drone_pose_cmd.yaw = 3.141592/2;

    

    ros::Time last_request = ros::Time::now();

    int count = 0;
    //Tag pose inertial frame
    float x_tag{0}, y_tag{0}, z_tag{0};
    //Tag pose body frame
    // float x_tag_body{0}, y_tag_body{0}, z_tag_body{0};
    //Local pose of drone
    float x_drone_feedback{0}, y_drone_feedback{0}, z_drone_feedback{0};
    //Command pose of drone
    float x_drone_cmd{0}, y_drone_cmd{0}, z_drone_cmd{0};
    float x_diff{0}, y_diff{0}, z_diff{0};
    while(ros::ok()){
        // if ((x_tag_body-tag_pose_body.pose.position.x==0) && (y_tag_body-tag_pose_body.pose.position.y==0) && (z_tag_body-tag_pose_body.pose.position.z==0)){
        //     continue;
        // }    
        //Transfer values from message to variable for april tag pose
        x_tag = g_pose.pose.position.x;
        y_tag = g_pose.pose.position.y;
        z_tag = g_pose.pose.position.z;

        //Transfer values from message to variable for drone pose
        x_drone_feedback = drone_pose_feedback.pose.position.x;
        y_drone_feedback = drone_pose_feedback.pose.position.y;
        z_drone_feedback = drone_pose_feedback.pose.position.z;

        //Calculate difference in positions in x and y
        x_diff = -x_drone_feedback+x_tag;
        y_diff = -y_drone_feedback+y_tag;

        //Control y movement of the drone
        if (y_diff>0.3){
            y_drone_cmd = y_drone_feedback+0.15;
        }
        else if (y_diff < 0.3){
            y_drone_cmd = y_drone_feedback-0.15;
        }

        //Control x movement of the drone
        if (x_diff>0){
            x_drone_cmd = x_drone_feedback+0.1;
        }
        else if (x_diff<0){
            x_drone_cmd = x_drone_feedback-0.1;
        }
        //Assign z same as drone's current height
        z_drone_cmd=0.55;

        //Build a command message for the drone's updated position
        drone_pose_cmd.position.x=x_drone_cmd;
        drone_pose_cmd.position.y=y_drone_cmd;
        drone_pose_cmd.position.z=z_drone_cmd;

        // std::cout<<"\nDiff in x: "<<x_diff;
        // std::cout<<"\nDiff in y: "<<y_diff;
        std::cout<<"\nx position command of drone: "<<x_drone_cmd;
        std::cout<<"\ny position command of drone: "<<y_drone_cmd;
        std::cout<<"\nz position command of drone: "<<z_drone_cmd;
        // std::cout<<"\nHeight of drone: "<<z_drone_cm

        //Publish the command message
        local_pos_pub_mavros.publish(drone_pose_cmd);

        ros::spinOnce();
        rate.sleep();
        count++;
	// std::cout << count << std::endl;

    }

    return 0;
}

