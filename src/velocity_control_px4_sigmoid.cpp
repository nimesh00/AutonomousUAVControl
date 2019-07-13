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
#include <std_msgs/Float64.h>
#include <math.h>
#include <vector>

using namespace std;

int x_initial = -10;
int y_initial = -10;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

bool got_new_coordinates = 0;

geometry_msgs::PoseStamped updated_coords;
geometry_msgs::PoseStamped last_coords;
void new_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    last_coords = updated_coords;
    updated_coords = *msg;
    got_new_coordinates = 1;
    if (!(last_coords.pose.position.x == updated_coords.pose.position.x &&
        last_coords.pose.position.y == updated_coords.pose.position.y &&
        last_coords.pose.position.z == updated_coords.pose.position.z)) {
            got_new_coordinates = 1;
        }
}

geometry_msgs::PoseStamped current_coordinates;
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates = *msg;
    current_coordinates.pose.position.x += x_initial;
    current_coordinates.pose.position.y += y_initial;
}

std_msgs::Float64 compass_current;
void compass_cb(const std_msgs::Float64::ConstPtr& msg) {
    compass_current = *msg;
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

float from360to180(float angle) {
    if (angle <= 180) {
        return angle;
    } else {
        return angle - 360;
    }
}

float sigmoid_modified(float x) {
    return 2 * (1.0 / (1 + exp(-x))) - 0.5;
}

// float tan_h(float x) {
//     return ((exp(2 * x) - 1.0) / (exp(2 * x) + 1.0));
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_control_px4_sigmoid");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("uav1/mavros/state", 10, state_cb);
    ros::Subscriber coords_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/updated_coordinates1", 10, new_coords_cb);
    ros::Subscriber current_coords_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("uav1/mavros/local_position/pose", 10, curr_coords_cb);
    ros::Subscriber compass = nh.subscribe<std_msgs::Float64>
            ("/uav1/mavros/global_position/compass_hdg", 10, compass_cb);
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

    geometry_msgs::PoseStamped global_pose, local_pose, last_coords;
    global_pose.pose.position.x = x_initial;
    global_pose.pose.position.y = y_initial;
    global_pose.pose.position.z = 2;

    // local_pose = global_pose;
    // local_pose.pose.position.x -= x_initial;
    // local_pose.pose.position.y -= y_initial;

    // //send a few setpoints before starting
    // for(int i = 100; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(local_pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    mavros_msgs::PositionTarget pose1;
    pose1.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;   //FRAME_LOCAL_NED=inertial frame        FRAME_BODY_NED = body frame
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

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    float distance_to_next = 0.0;
    float angle = 0.0;
    float correction_rate = 0.0;
    float velocity_factor = 5;
    
    int wp_no = 0;

    std_msgs::Float64 compass_initial = compass_current;

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
            last_coords = global_pose;
            global_pose = updated_coords;
            // cout << "Got new Coordinates" << global_pose.pose.position.x << global_pose.pose.position.y << global_pose.pose.position.z << endl;
            got_new_coordinates = 0;
        }

        // cout << "Total waypoints: " << sizeof(wayPoints) / sizeof(geometry_msgs::PoseStamped) << endl;

        // geometry_msgs::PoseStamped quarternions = get_quarternions(current_coordinates, global_pose);

        distance_to_next = distance_between_cartesian_points(global_pose, current_coordinates);
        float angle = get_angle(current_coordinates, global_pose);
        float angle_degrees = angle * (180.0 / M_PI);
        float current_angle = compass_initial.data - compass_current.data;
        current_angle = from360to180(current_angle);
        float delta_angle = angle_degrees - current_angle;
        delta_angle = delta_angle * (M_PI / 180);

        cout << "distance to next waypoint: " << distance_to_next << endl;


        float delta_x = global_pose.pose.position.x - current_coordinates.pose.position.x;
        float delta_y = global_pose.pose.position.y - current_coordinates.pose.position.y;
        float delta_z = global_pose.pose.position.z - current_coordinates.pose.position.z;

        if (!((distance_to_next - abs(delta_z)) < 0.2)) {
            correction_rate = 2 * sigmoid_modified(delta_angle / 3);
        } else {
            correction_rate = 0;
        }

        if (distance_to_next < 0.2) {
            velocity_factor = 1;
        } else {
            velocity_factor = 3;
        }
        
        cout << "Target angle: " << angle_degrees << endl;
        cout << "Current Angle: " << current_angle << endl;
        cout << "Delta Angle: " << delta_angle << endl;

        // correction_rate = 2 * tanhl(delta_angle / 3);

        pose1.yaw_rate = correction_rate;


        cout << "current target: " << global_pose.pose.position.x << global_pose.pose.position.y << global_pose.pose.position.z << endl;

        float unit_vec_x = delta_x / distance_to_next;
        float unit_vec_y = delta_y / distance_to_next;
        float unit_vec_z = delta_z / distance_to_next;

        pose1.velocity.x = velocity_factor * sigmoid_modified(unit_vec_x);
        pose1.velocity.y = velocity_factor * sigmoid_modified(unit_vec_y);
        pose1.velocity.z = velocity_factor * sigmoid_modified(unit_vec_z);
        // cout << "Deltas: " << "\tX: " << delta_x << "\tY: " << delta_y << "\tZ: " << delta_z << endl;
        cout << "Correction Rate: " << correction_rate << endl;
        cout << "Velocites: " << "\tX: " << pose1.velocity.x << "\tY: " << pose1.velocity.y << "\tZ: " << pose1.velocity.z << endl;

        local_bodyFrame_pos_pub.publish(pose1);
        // local_pos_pub.publish(local_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}