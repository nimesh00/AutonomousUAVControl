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

geometry_msgs::PoseStamped current_coordinates;
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_coordinates = *msg;
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

    ros::init(argc, argv, "new_waypoint_publisher");
    ros::NodeHandle nh;

    // ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, home_cb);
    ros::Publisher home_pos_pub = nh.advertise<mavros_msgs::HomePosition>("/mavors/home_position/set", 10);
    ros::Publisher updated_coordinates = nh.advertise<geometry_msgs::PoseStamped>("/updated_coordinates", 10);
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

    while(ros::ok()) {
        home_pos_pub.publish(new_home);
        float *next_coordinates = (float*)malloc(sizeof(geo_points[0]));

        next_coordinates = geo_points[i];
        cartesian_coords *next_cartesian_points = change_geodetic_to_cartesian(next_coordinates[0], next_coordinates[1], new_home.geo.altitude);
        cartesian_coords *local_coords = (cartesian_coords*)malloc(sizeof(cartesian_coords));

        local_coords->x = next_cartesian_points->x - home_cartesian->x;
        local_coords->y = next_cartesian_points->y - home_cartesian->y;
        local_coords->z = next_cartesian_points->z - home_cartesian->z;

        geometry_msgs::PoseStamped new_cartesian_coords;

        new_cartesian_coords.header.stamp = ros::Time::now();
        // new_cartesian_coords.pose.position.x = local_coords->x;
        // new_cartesian_coords.pose.position.y = local_coords->y;
        // new_cartesian_coords.pose.position.z = local_coords->z;

        new_cartesian_coords.pose.position.x = local_points[i][0];
        new_cartesian_coords.pose.position.y = local_points[i][1];
        new_cartesian_coords.pose.position.z = local_points[i][2];

        cout << "New waypoint given: " << new_cartesian_coords.pose.position.x
                                        << new_cartesian_coords.pose.position.y
                                        << new_cartesian_coords.pose.position.z << endl;

        updated_coordinates.publish(new_cartesian_coords);
        distance_to_current = distance_between_cartesian_points(new_cartesian_coords, current_coordinates);

        if (distance_to_current <= 5) {
            i = (i + 1) % 4;
        }
        // i = (i + 1) % 4;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}