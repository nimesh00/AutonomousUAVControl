/*
* @file new_waypoint_calc.cpp
* Converts global waypoints (Lat, Lon, Alt) to Local waypoints (x, y, z) and sends over the /updated_coordinates topic
*/

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <math.h>
#include <stdlib.h>

using namespace std;

mavros_msgs::HomePosition new_home;

#define PI 3.14159265
#define a 6378137.0
#define b 6356752.3

//Defining the dimensions of the map in which the UAV operates
int xmax=10, ymax=10;
//Matrix to hold the entire waypoints generated for the lawn mover movement
int lawn_mower_matrix[100][3];

geometry_msgs::PoseStamped current_coordinates;
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates = *msg;
}

void lawn_mower_pattern_generator(int xmax, int ymax) {
    int  i = 0, j = 0;
    int x_use = -2, y_use = 0;

    for (i = 0; i < xmax * ymax; i++) {
        if (i % 10 == 0) {
            x_use += 2;
        }
        for (j = 0; j < 3; j++) {
            if (j == 0) 
            {
                lawn_mower_matrix[i][j] = x_use;
            } 
            else if (j == 1)
             {
                lawn_mower_matrix[i][j] = (x_use % 4 == 0) ? (i % 10) : (xmax - (i % 10) - 1);
            } else {
                lawn_mower_matrix[i][j] = 10;
            }
            cout << lawn_mower_matrix[i][j];
        }
        cout << endl;
        
    }
}



typedef struct cartesian_coordinates_struct {
    float x;
    float y;
    float z;
} cartesian_coords;

float the_N_term(float angle) {
    float e_2 = 1 - (b * b) / (a * a);
    return (a / sqrt(1 - e_2 * (sin(angle) * sin(angle))));
}

cartesian_coords* change_geodetic_to_cartesian(float latitude, float longitude, float altitude) {
    cartesian_coords* point = (cartesian_coords*)malloc(sizeof(cartesian_coords));
    float fi = latitude * PI / 180;
    float lambda = longitude * PI / 180;
    (*point).x = (the_N_term(fi) + altitude) * cos(fi) * cos(lambda);
    (*point).y = (the_N_term(fi) + altitude) * cos(fi) * sin(lambda);
    (*point).z = ((pow(b, 2) / pow(a, 2)) * the_N_term(fi) + altitude) * sin(fi);
    return point;
}

float distance_between_cartesian_points(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2) {
    float delta_x = point1.pose.position.x - point2.pose.position.x;
    float delta_y = point1.pose.position.y - point2.pose.position.y;
    float delta_z = point1.pose.position.z - point2.pose.position.z;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "lawn_mower");
    ros::NodeHandle nh;

    // ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, home_cb);
    ros::Publisher home_pos_pub = nh.advertise<mavros_msgs::HomePosition>("/mavors/home_position/set", 10);
    ros::Publisher updated_coordinates = nh.advertise<geometry_msgs::PoseStamped>("/updated_coordinates1", 10);
    ros::Publisher chatter_pub = nh.advertise<geometry_msgs::PoseStamped>("/updated_coordinates", 10);
    ros::Subscriber current_coords_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose", 10, curr_coords_cb);

    ros::Rate rate(10.0);

    new_home.header.stamp = ros::Time::now();
    new_home.geo.latitude = 13.0257405;
    new_home.geo.longitude = 77.5666214;
    new_home.geo.altitude = 1350;

    home_pos_pub.publish(new_home);

    cartesian_coords *home_cartesian = change_geodetic_to_cartesian(new_home.geo.latitude,
                                                                    new_home.geo.longitude,
                                                                    new_home.geo.altitude);

    
    float geo_points[4][2] = {{13.027439, 77.563226}, {13.027324, 77.564546}, {13.026796, 77.564435}, {13.027029, 77.563604}};
    // float local_points[4][3] = {{75.625, -24.375, 20}, {-63.375, 9.625, 20}, {-49.625, 20.125, 20}, {37.375, -5.375, 20}};
    float local_points[4][3] = {{20, 0, 7}, {20, 20, 7}, {20, 10, 7}, {-10, 20, 7}};
    int i = 0;

    float distance_to_current = 0.0;
    float cartestian_distance = 0.0;
    lawn_mower_pattern_generator(xmax, ymax);

    while(ros::ok()) {
        home_pos_pub.publish(new_home);
        float *next_coordinates = (float*)malloc(sizeof(geo_points[0]));

        next_coordinates = geo_points[i];
        cartesian_coords *next_cartesian_points = change_geodetic_to_cartesian(next_coordinates[0], next_coordinates[1], new_home.geo.altitude);
        cartesian_coords *local_coords = (cartesian_coords*)malloc(sizeof(cartesian_coords));

        local_coords->x = next_cartesian_points->x - home_cartesian->x;
        local_coords->y = next_cartesian_points->y - home_cartesian->y;
        local_coords->z = next_cartesian_points->z - home_cartesian->z;

        // geometry_msgs::PoseStamped new_cartesian_coords;
        geometry_msgs::PoseStamped coords_msg;

        coords_msg.header.stamp = ros::Time::now();
    
        coords_msg.pose.position.x = (float)lawn_mower_matrix[i][0];
        coords_msg.pose.position.y = (float)lawn_mower_matrix[i][1];
        coords_msg.pose.position.z = (float)lawn_mower_matrix[i][2];
        cartestian_distance = distance_between_cartesian_points(current_coordinates, coords_msg);
        // ROS_INFO("I sent: [%.2f] X Coordinate and [%.2f] Y Coordinate\n", coords_msg.pose.position.x, coords_msg.pose.position.y);
        // cout << coords_msg;
        cout << cartestian_distance << endl;
        chatter_pub.publish(coords_msg);
        if (cartestian_distance <= 0.5)
         {
            i = (i + 1) % (xmax*ymax);
        }

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}