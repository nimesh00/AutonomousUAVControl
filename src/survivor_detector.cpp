/*
* @file new_waypoint_calc.cpp
* Converts global waypoints (Lat, Lon, Alt) to Local waypoints (x, y, z) and sends over the /updated_coordinates topic
*/

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <stdlib.h>

using namespace std;

// initial position of the UAV
int x_initial = -10;
int y_initial = -10;

//Defining the dimensions of the map in which the UAV operates
#define xmax 20
#define ymax 20

//Matrix to hold the entire waypoints generated for the lawn mover movement
const int total_waypoints = xmax * ymax;
int lawn_mower_matrix[total_waypoints][3];

geometry_msgs::PoseStamped current_coordinates;
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates = *msg;
    current_coordinates.pose.position.x += x_initial;
    current_coordinates.pose.position.y += y_initial;
}


nav_msgs::Odometry survivor_location_new;
nav_msgs::Odometry survivor_location_last;
bool got_new_location = 0, observer_publishing = 0;;
void observer_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    observer_publishing = 1;
    survivor_location_last = survivor_location_new;
    survivor_location_new = *msg;
    if ( !(msg->pose.pose.position.x == survivor_location_last.pose.pose.position.x &&
        msg->pose.pose.position.y == survivor_location_last.pose.pose.position.y &&
        msg->pose.pose.position.z == survivor_location_last.pose.pose.position.z)) {
        got_new_location = 1;
    }
}

void lawn_mower_pattern_generator() {
    int  i = 0, j = 0;
    int x_use = -2, y_use = 0;
    int x_drift = 2;

    for (i = 0; i < xmax * ymax; i++) {
        if (i % xmax == 0) {
            x_use += x_drift;
        }
        for (j = 0; j < 3; j++) {
            if (j == 0) 
            {
                lawn_mower_matrix[i][j] = x_use + x_initial;
            } 
            else if (j == 1)
             {
                lawn_mower_matrix[i][j] = ((x_use % (2 * x_drift) == 0) ? (i % ymax) : (ymax - (i % ymax) - 1)) + y_initial;
            } else {
                lawn_mower_matrix[i][j] = 10;
            }
            cout << lawn_mower_matrix[i][j];
        }
        cout << endl;
        
    }
}

const int n_obsrvr = 2;
int observer_radius = 2;
int observer_locations[n_obsrvr][2] = {{-5, 5}, {6, -6}};

typedef struct cartesian_coordinates_struct {
    float x;
    float y;
    float z;
} cartesian_coords;

float distance_between_cartesian_points(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_x = point1.pose.position.x - point2.pose.position.x;
    float delta_y = point1.pose.position.y - point2.pose.position.y;
    float delta_z = point1.pose.position.z - point2.pose.position.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
}

bool inside_observer_radius(nav_msgs::Odometry location) {
    int i = 0;
    float distances[n_obsrvr] = {0.0, 0.0};
    float current_location[2] = {0.0, 0.0};
    current_location[0] = location.pose.pose.position.x;
    current_location[1] = location.pose.pose.position.y;
    
    for (i = 0; i < n_obsrvr; i++) {
        distances[i] = sqrt(pow((current_location[0] - observer_locations[i][0]), 2) + pow((current_location[1] - observer_locations[i][1]), 2));
        if (distances[i] < observer_radius){
            return 1;
        }
    }
    return 0;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "lawn_mower");
    ros::NodeHandle nh;

    // ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, home_cb);
    ros::Publisher home_pos_pub = nh.advertise<mavros_msgs::HomePosition>("/mavors/home_position/set", 10);
    ros::Publisher updated_coordinates = nh.advertise<geometry_msgs::PoseStamped>("/updated_coordinates", 10);
    ros::Publisher chatter_pub = nh.advertise<geometry_msgs::PoseStamped>("/updated_coordinates1", 10);
    ros::Subscriber current_coords_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose", 10, curr_coords_cb);
    ros::Subscriber observer_sub = nh.subscribe<nav_msgs::Odometry>("/observer/odom", 10, observer_cb);

    ros::Rate rate(10.0);

    int i = 0;
    float distance_to_current = 0.0;
    float cartestian_distance = 0.0;
    lawn_mower_pattern_generator();

    geometry_msgs::PoseStamped waypoints_to_publish;

    while(ros::ok()) {
        // geometry_msgs::PoseStamped new_cartesian_coords;
        geometry_msgs::PoseStamped coords_msg;

        // cout << "New Coordinate status" << got_new_location;

        if (got_new_location == 0) {
            cout << "Going lawn mower!!" << endl;
            coords_msg.header.stamp = ros::Time::now();
        
            coords_msg.pose.position.x = (float)lawn_mower_matrix[i][0];
            coords_msg.pose.position.y = (float)lawn_mower_matrix[i][1];
            coords_msg.pose.position.z = (float)lawn_mower_matrix[i][2];
            cartestian_distance = distance_between_cartesian_points(current_coordinates, coords_msg);
            // ROS_INFO("I sent: [%.2f] X Coordinate and [%.2f] Y Coordinate\n", coords_msg.pose.position.x, coords_msg.pose.position.y);
            // cout << coords_msg;
            cout << cartestian_distance << endl;
            if (cartestian_distance <= 0.5)
            {
                i = (i + 1) % (xmax*ymax);
            }

            waypoints_to_publish = coords_msg;

        } else if (got_new_location == 1) {
            geometry_msgs::PoseStamped target_waypoint;
            // if (survivor_location_new.pose.pose.position.x == survivor_location_last.pose.pose.position.x &&
            //     survivor_location_new.pose.pose.position.y == survivor_location_last.pose.pose.position.y &&
            //     survivor_location_new.pose.pose.position.z == survivor_location_last.pose.pose.position.z) {

            if (observer_publishing) {                    
                cout << "Survivor inside range!!" << endl;
                target_waypoint.pose.position.x = survivor_location_new.pose.pose.position.x;
                target_waypoint.pose.position.y = survivor_location_new.pose.pose.position.y;
                target_waypoint.pose.position.z = 10;
                waypoints_to_publish = target_waypoint;
            } else {
                cout << "predict the new location!!" << endl;
                
            }
            
        }

        

        chatter_pub.publish(waypoints_to_publish);
        

        observer_publishing = 0;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}