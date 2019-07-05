/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool got_new_coordinates = 0;

geometry_msgs::PoseStamped updated_coords;
geometry_msgs::PoseStamped last_coords;
vector<geometry_msgs::PoseStamped::ConstPtr> updated_coords_msgs;
void new_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_coords = updated_coords;
    updated_coords = *msg;
    if (!(last_coords.pose.position.x == updated_coords.pose.position.x &&
        last_coords.pose.position.y == updated_coords.pose.position.y &&
        last_coords.pose.position.z == updated_coords.pose.position.z)) {
            updated_coords_msgs.push_back(msg);
            got_new_coordinates = 1;
        }
}

geometry_msgs::PoseStamped current_coordinates;
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates = *msg;
}

float distance_between_cartesian_points(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_x = point1.pose.position.x - point2.pose.position.x;
    float delta_y = point1.pose.position.y - point2.pose.position.y;
    float delta_z = point1.pose.position.z - point2.pose.position.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
}

geometry_msgs::PoseStamped get_quarternions(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_y = point2.pose.position.y - point1.pose.position.y;
    float delta_x = point2.pose.position.x - point1.pose.position.x;

    float angle = atan2(delta_y, delta_x);
    geometry_msgs::PoseStamped return_value;
    return_value.pose.orientation.w = cos(angle / 2);
    return_value.pose.orientation.z = sin(angle / 2);
    return_value.pose.orientation.y = 0;
    return_value.pose.orientation.x = 0;
    return return_value;
}

float get_angle(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_y = point2.pose.position.y - point1.pose.position.y;
    float delta_x = point2.pose.position.x - point1.pose.position.x;

    return atan2(delta_y, delta_x);
}

geometry_msgs::PoseStamped delta_vectors(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    geometry_msgs::PoseStamped delta_vector;
    delta_vector.pose.position.z = point2.pose.position.z - point1.pose.position.z;
    delta_vector.pose.position.y = point2.pose.position.y - point1.pose.position.y;
    delta_vector.pose.position.x = point2.pose.position.x - point1.pose.position.x;
    return delta_vector;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Subscriber coords_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/updated_coordinates", 10, new_coords_cb);
    ros::Subscriber current_coords_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, curr_coords_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("uav1/mavros/setpoint_position/local", 10);
    ros::Publisher local_bodyFrame_pos_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("uav1/mavros/setpoint_raw/local", 1);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("uav1/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget pose1;
    pose1.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;   //FRAME_LOCAL_NED=inertial frame        FRAME_BODY_NED = body frame
    pose1.header.frame_id = "world";
    pose1.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                     mavros_msgs::PositionTarget::IGNORE_PY |
                     mavros_msgs::PositionTarget::IGNORE_PZ |
                     mavros_msgs::PositionTarget::IGNORE_AFX |
                     mavros_msgs::PositionTarget::IGNORE_AFY |
                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                     mavros_msgs::PositionTarget::FORCE |
                     mavros_msgs::PositionTarget::IGNORE_YAW;
    pose1.header.stamp = ros::Time::now();

    // pose1.position.x = 0;
    // pose1.position.y = 0;
    // pose1.position.z = 2;
    // pose1.yaw_rate = 1;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    float distance_to_next = 0.0;
    float angle = 0.0;
    float correction_rate = 2.0;
    
    // cout << "starting menory allocation!!" << endl;
    // geometry_msgs::PoseStamped *wayPoints = (geometry_msgs::PoseStamped*)malloc(sizeof(geometry_msgs::PoseStamped));
    // if (wayPoints == NULL) {
    //     cout << "couldn't initialize waypoint memory block!" << endl;
    // } else {
    //     cout << "memory initalized successfully" << endl;
    // }
    int wp_no = 0;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (got_new_coordinates == 1) {

            cout << "Got new coordinates!" << endl;

            // static int current_size = sizeof(wayPoints) / sizeof(geometry_msgs::PoseStamped);

            // cout << "Current size: " << current_size;
            
            // wayPoints = (geometry_msgs::PoseStamped*)realloc(wayPoints, (current_size + 1) * sizeof(geometry_msgs::PoseStamped));
            // wayPoints = (geometry_msgs::PoseStamped*)realloc(wayPoints, (current_size + 1) * sizeof(*wayPoints));
            // if (wayPoints == NULL) {
            //     cout << "Problem resizing the memory block!" << endl;
            // } else {
            //     cout << "Resizing done successfully" << endl;
            // }
            // // cout << *wayPoints << endl;
            // // cout << wayPoints[current_size] << endl;
            // // *(wayPoints + current_size) = updated_coords;
            got_new_coordinates = 0;
        }
        if (distance_to_next <= 0.5) {
            // pose = updated_coords;
            cout << *updated_coords_msgs[wp_no];
            cout << updated_coords_msgs.size();
            pose = *updated_coords_msgs[wp_no++];
            // pose1.position.x = pose.pose.position.x;
            // pose1.position.y = pose.pose.position.y;
            // pose1.position.z = pose.pose.position.z;
            // pose1.header.stamp = ros::Time::now();
            cout << "Got new Coordinates" << pose.pose.position.x << pose.pose.position.y << pose.pose.position.z << endl;    
        }

        // cout << "Total waypoints: " << sizeof(wayPoints) / sizeof(geometry_msgs::PoseStamped) << endl;

        geometry_msgs::PoseStamped quarternions = get_quarternions(current_coordinates, pose);

        float angle = get_angle(current_coordinates, pose);

        distance_to_next = distance_between_cartesian_points(pose, current_coordinates);
        cout << "distance to nexy waypoint: " << distance_to_next << endl;

        
        float delta_z = abs(pose.pose.position.z - current_coordinates.pose.position.z);

        if (!((distance_to_next - delta_z) < 1)) {
            pose.pose.orientation.z = quarternions.pose.orientation.z;
            pose.pose.orientation.w = quarternions.pose.orientation.w;
            // pose1.yaw = angle;
            // pose1.yaw_rate = angle * (correction_rate / M_PI);    
        }
        
        // local_bodyFrame_pos_pub.publish(pose1);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}