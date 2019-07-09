#!/usr/bin/env python
import rospy
import roslaunch 
import time

def start_task():
	rospy.loginfo("starting...")

	package1="uavmanualcontrol"
	executable1="mbzirc_ch2_navigation"
	# executable2="ball_2.4_ros_gazebo_uav2.py"
	# executable2="ball_detect_track_destination_ros_with_pos.py"
	# executable3="ball_2.4_ros_gazebo_uav3.py"
	executable4="body_frame_teleop_uav1.py"
	executable5="body_frame_teleop_uav2.py"
	executable6="body_frame_teleop_uav3.py"
	node1 = roslaunch.core.Node(package1, executable1)
	# node2 = roslaunch.core.Node(package1, executable2)
	# node3 = roslaunch.core.Node(package1, executable3)
	node4 = roslaunch.core.Node(package1, executable4)
	node5 = roslaunch.core.Node(package1, executable5)
	node6 = roslaunch.core.Node(package1, executable6)

	launch = roslaunch.scriptapi.ROSLaunch()
	launch.start()

	# script1 = launch.launch(node1)
	# print script1.is_alive()
	script6 = launch.launch(node6)
	print script6.is_alive()
	script5 = launch.launch(node5)
	print script5.is_alive()
	# script3 = launch.launch(node3)
	# print script3.is_alive()
	script4 = launch.launch(node4)
	print script4.is_alive()
	# script2 = launch.launch(node2)
	# print script2.is_alive()
	# time.sleep(5)
	
	
	
def main():
	rospy.init_node('main_node')
	start_task()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

