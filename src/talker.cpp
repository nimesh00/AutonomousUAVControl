#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
using namespace std;


//Defining the dimensions of the map in which the UAV operates
int xmax=10, ymax=10;
//Matrix to hold the entire waypoints generated for the lawn mover movement
int lawn_mower_matrix[10][10][2];

//Coordinates of the UAV
geometry_msgs::PoseStamped current_coordinates, coords_msg;
void curr_coords_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cout<<"Reached curr cords"<<endl;
  //Current coordinates of the UAV, points to MSG as a callback    
    cout << msg->pose.position.x;
    current_coordinates = *msg;
}

float distance_between_cartesian_points(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2)
{
    cout<<"Reached distance_between"<<endl;
    //Calculating the cartesian distance between current UAV position and given waypoint
    float delta_x = point1.pose.position.x - point2.pose.position.x;
    float delta_y = point1.pose.position.y - point2.pose.position.y;
    float delta_z = 5;
    return sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
}

void lawn_mower_pattern_generator(int xmax, int ymax)
{
  cout<<"Reached lawn mower generator"<<endl;
  //Generating the lawn mower pattern for initial traversal
  for (int count_i = 1; count_i<=xmax; count_i++)
  {
    //Since direction of pattern has to change, we use these conditions. Alternate rows have different directions of traversal
    if((count_i%2)==0)
    {

        for (int count_j = 1; count_j<=ymax; count_j++)
          {
            lawn_mower_matrix[count_i][count_j][0] = count_i;
            lawn_mower_matrix[count_i][count_j][1] = count_j;
           // cout<<lawn_mower_matrix[count_i][count_j][0]<<lawn_mower_matrix[count_i][count_j][1]<<endl;
          }
    }
  else
    {
      for (int count_j = ymax; count_j>=1; count_j--)
        {
          lawn_mower_matrix[count_i][count_j][0] = count_i;
          lawn_mower_matrix[count_i][count_j][1] = count_j;
        //    cout<<lawn_mower_matrix[count_i][count_j][0]<<lawn_mower_matrix[count_i][count_j][1]<<endl;
        }
    }
  }
  
}

int main(int argc, char **argv)
{ 
  cout << "Reaching here!" << endl;
  //Initializes ROS and node of unique name
  ros::init(argc, argv, "talker");
  //Kills previously initialized nodes of the same name if running
  ros::NodeHandle n;
  //Subscribing to the current coordinates of the UAV
  ros::Subscriber current_coords_sub = n.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, curr_coords_cb);
  //Publishes to the updated coordinates which the UAV will subscribe to
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/updated_coordinates", 10);
  //Frequency of information publications, since last sleep; Currently 10 Hz
  ros::Rate loop_rate(10);
  //Declaring the type of message that we are publishing using the publisher written here

  //Simple counter variables to loop through the entire 10*10 map

  float cartestian_distance;
  lawn_mower_pattern_generator(xmax, ymax);
  cout<<"Post lawn mower"<<endl;
  while (ros::ok()) {
    for (int count_i=1; count_i<=xmax;)
    {
        for (int count_j=1; count_j<=ymax;)
        {
          //Sending the X and the Y coordinates via pose
          coords_msg.pose.position.x = lawn_mower_matrix[count_i][count_j][0];
          coords_msg.pose.position.y = lawn_mower_matrix[count_i][count_j][1];
          cout << coords_msg;
          cout<<"Pre cartesian distance"<<endl;
          cartestian_distance = distance_between_cartesian_points(current_coordinates, coords_msg);
          
          if(cartestian_distance<=0.5)
            {
              count_j++;
            }
          
          chatter_pub.publish(coords_msg);
          //Publishing the entire packet to 
          
          ROS_INFO("I sent: [%.2f] X Coordinate and [%.2f] Y Coordinate\n", coords_msg.pose.position.x, coords_msg.pose.position.y);
        }
        
        if(cartestian_distance<=0.5)
          {
            count_i++;
          }
      }
      ros::spinOnce();
          loop_rate.sleep();
  }
  return 0;
}