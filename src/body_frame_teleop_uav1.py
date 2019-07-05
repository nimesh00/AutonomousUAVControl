#!/usr/bin/env python

from __future__ import print_function

#import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy


from std_msgs.msg import Header, Float32, Float64, Empty
from geometry_msgs.msg import Twist, PoseStamped , Point

import sys, select, termios, tty

import mavros
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import PositionTarget
from mavros import setpoint as SP
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
import sys,signal, os

import threading, time

desired_x = desired_y = desired_z = 0
des_pos_x = des_pos_y = des_pos_z = 0
desired_pitch = desired_roll = desired_yaw = 0
current_x = current_y = current_z = 0

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

0 to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		'o':(1,0,0,-1),
		'j':(0,0,0,1),
		'l':(0,0,0,-1),
		'u':(1,0,0,1),
		',':(-1,0,0,0),
		'.':(-1,0,0,1),
		'm':(-1,0,0,-1),
		'O':(1,-1,0,0),
		'I':(1,0,0,0),
		'J':(0,1,0,0),
		'L':(0,-1,0,0),
		'U':(1,1,0,0),
		'<':(-1,0,0,0),
		'>':(-1,-1,0,0),
		'M':(-1,1,0,0),
		't':(0,0,1,0),
		'b':(0,0,-1,0),
	       }

speedBindings={
		'q':(1.1,1.1),
		'z':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
	      }
def isData():
	return select.select([sys.stdin], [], [], 0.05) == ([sys.stdin], [], [])
def getKey():
	tty.setraw(sys.stdin.fileno())
	# select.select([sys.stdin], [], [], 0)
	key = None
	if isData():
		key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def twist_obj(x, y, z, a, b, c):
    # move_cmd = Twist()
	move_cmd = PositionTarget()
	move_cmd.velocity.x = x
	move_cmd.velocity.y = y
	move_cmd.velocity.z = z
	# move_cmd.yaw_rate = c
	move_cmd.header.stamp = rospy.get_rostime()
	move_cmd.header.frame_id ="world"
	move_cmd.coordinate_frame =8
	move_cmd.type_mask = 3527
	return move_cmd


def arming():
	global args
	state = True
	pub1.publish(twist_obj(0,0,0,0,0,0))
	try:
		arming_cl = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
		ret = arming_cl(value=state)
	except rospy.ServiceException as ex:
		fault(ex)

def handle_poses(data):
	current_x=data.pose.pose.position.x
	current_y=data.pose.pose.position.y
	current_z=data.pose.pose.position.z


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)


def persistVel():
    global pub1
    global body
    t = threading.currentThread()
    # while getattr(t, "do_run", True):
    pub1.publish(body)
    time.sleep(0.01)

current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

def callback(data):
	global desired_x,desired_y,desired_z,desired_pitch,desired_roll,desired_yaw

	desired_x= data.linear.x
	desired_y= data.linear.y
	desired_z= data.linear.z
	desired_pitch= data.angular.y
	desired_roll= data.angular.x
	desired_yaw= data.angular.z

def takeoff():
	mode = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
	resp = mode(0,"AUTO.TAKEOFF")

def pos_callback(data):
	global des_pos_x, des_pos_y, des_pos_z

	des_pos_x= data.pose.position.x
	des_pos_y= data.pose.position.y
	des_pos_z= data.pose.position.z

pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 2

def position_control():
    prev_state = current_state
    rate = rospy.Rate(100.0) # MUST be more then 2Hz

    # send a few setpoints before starting
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    # # wait for FCU connection
    # while not current_state.connected:
    #     rate.sleep()

    last_request = rospy.get_rostime()
    # while not rospy.is_shutdown():
    now = rospy.get_rostime()
    if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
        set_mode_client(base_mode=0, custom_mode='OFFBOARD')
        last_request = now 
    else:
        if not current_state.armed and (now - last_request > rospy.Duration(5.)):
           arming_client(True)
           last_request = now 

    # older versions of PX4 always return success==True, so better to check Status instead
    if prev_state.armed != current_state.armed:
        rospy.loginfo("Vehicle armed: %r" % current_state.armed)
    if prev_state.mode != current_state.mode: 
        rospy.loginfo("Current mode: %s" % current_state.mode)
    prev_state = current_state

    # Update timestamp and publish pose 
    pose.header.stamp = rospy.Time.now()
    local_pos_pub.publish(pose)
    rate.sleep()


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
	
	# mavros.set_namespace("/uav1/mavros")
	pub = rospy.Publisher("/uav1/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=100)
	pub1 = rospy.Publisher("/uav1/mavros/setpoint_raw/local",PositionTarget,queue_size=100)
	local_pos_pub = rospy.Publisher("/uav1/mavros/setpoint_position/local", PoseStamped, queue_size=100)
	state_sub = rospy.Subscriber("/uav1/mavros/state", State, state_cb)
	uav_pos=rospy.Subscriber("/uav1/mavros/global_position/local",Odometry,handle_poses)
	arming_client = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
	set_mode_client = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode) 
	rospy.Subscriber("chatter1", Twist, callback)
	rospy.Subscriber("setpoint_pos1", PoseStamped, pos_callback)
	rospy.init_node('teleop_twist_keyboard1')

	position_control()
	mode = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
	resp = mode(base_mode=0,custom_mode='offboard')
	arming()
	speed = rospy.get_param("~speed", 0.5)
	turn = rospy.get_param("~turn", 1.0)
	x = 0
	y = 0
	z = 0
	th = 0
	status = 0


	body = PositionTarget()
	body.velocity.x = 0; body.velocity.y = 0; body.velocity.z = 0;
	# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
	body.header.stamp = rospy.get_rostime()
	body.header.frame_id ="world"
	body.coordinate_frame =8
	body.type_mask = 3527
	pub1.publish(body)

	pos = PoseStamped()
	pos.pose.position.x=0;pos.pose.position.y=0;pos.pose.position.z=0;
	pos.pose.orientation.x=0;pos.pose.orientation.y=0;pos.pose.orientation.z=0;pos.pose.orientation.w=0;
	T = threading.Thread(target = persistVel)
	T.start()
	#####################################################teleop mode
	try:
		print(msg)
		print(vels(speed,turn))
		while not rospy.is_shutdown():
			key = getKey()
			if key in moveBindings.keys():
				x = -1*moveBindings[key][1]
				y = moveBindings[key][0]
				z = moveBindings[key][2]
				th = moveBindings[key][3]


			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]

				print(vels(speed,turn))
				if (status == 14):
					print(msg)
				status = (status + 1) % 15

			elif(key == 'y'):
			    print("Armed")
			    arming()

			elif(key == '0'):
			    print("Exiting")
			    T.do_run = False
			    T.join()
			    sys.exit(0)

			else:
				x = 0
				y = 0
				z = 0
				th = 0
				if (key == '\x03'):
					break

			body = PositionTarget()
			body.velocity.x = x*speed; body.velocity.y = y*speed; body.velocity.z = z*speed;
			body.yaw_rate = th*turn
			# body.yaw = 1
			body.header.stamp = rospy.get_rostime()
			body.header.frame_id ="world"
			body.coordinate_frame =8
			body.type_mask = int('011111000111', 2)
			pub1.publish(body)

	except Exception as e:
		print(e)

	finally:
		body = PositionTarget()
		body.velocity.x = 0; body.velocity.y = 0; body.velocity.z = 0;
		body.yaw_rate =th*turn
		# body.yaw = 1
		# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
		body.header.stamp = rospy.get_rostime()
		body.header.frame_id ="world"
		body.coordinate_frame =8
		body.type_mask = int('011111000111', 2)
		pub1.publish(body)
		T.join()
    	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

###############################################################################control mode
	# try:
	# 	for i in range(15):
	# 		# print(msg)
	# 		rate = rospy.Rate(20.0)
	# 		x=0
	# 		y=0
	# 		z=5
	# 		th = 0
	# 		speed = 2
	# 		turn = 0
	# 		body = PositionTarget()
	# 		body.velocity.x = x*speed; body.velocity.y = y*speed; body.velocity.z = z*speed;
	# 		# body.yaw_rate = th*turn
	# 		body.head1/mavros/cmer.stamp = rospy.get_rostime()
	# 		body.header.frame_id ="world"
	# 		body.coordinate_frame =8
	# 		body.type_mask = int('011111000111', 2)
	# 		pub1.publish(body)
	# 		# print(vels(speed,turn))
	# 		rate.sleep()
	# 		x=0
	# 		y=0
	# 		z=0
	# 		th=0
	# 		body = PositionTarget()
	# 		body.velocity.x = x*speed; body.velocity.y = y*speed; body.velocity.z = z*speed;
	# 		# body.yaw_rate = th*turn
	# 		body.header.stamp = rospy.get_rostime()
	# 		body.header.frame_id ="world"
	# 		body.coordinate_frame =8
	# 		body.type_mask = int('011111000111', 2)
	# 		pub1.publish(body)
	# 		# print(vels(speed,turn))
	# 		# takeoff()
	# 	print(msg)
	# 	print(vels(speed,turn))
	# 	while not rospy.is_shutdown():
	# 		if desired_pitch==1:
	# 			x = desired_x
	# 			y = desired_y 
	# 		else:
	# 			x = -1*desired_y
	# 			y = desired_x
	# 		z = desired_z
	# 		th = desired_yaw
			
	# 		speed = 1
	# 		turn = 1

	# 		body = PositionTarget()
	# 		body.velocity.x = x*speed; body.velocity.y = y*speed; body.velocity.z = z*speed;
	# 		body.yaw_rate = th*turn
	# 		body.header.stamp = rospy.get_rostime()
	# 		body.header.frame_id ="world"
	# 		if desired_pitch==1:
	# 			body.coordinate_frame =1
	# 		else:
	# 			body.coordinate_frame =8
	# 		body.type_mask = int('011111000111', 2)
	# 		pub1.publish(body)

	# except Exception as e:
	# 	print(e)

	# finally:
	# 	body = PositionTarget()
	# 	body.velocity.x = 0; body.velocity.y = 0; body.velocity.z = 0;
	# 	body.yaw_rate = 0
	# 	body.header.stamp = rospy.get_rostime()
	# 	body.header.frame_id ="world"
	# 	if desired_pitch==1:
	# 		body.coordinate_frame =1
	# 	else:
	# 		body.coordinate_frame =8
	# 	body.type_mask = int('011111000111', 2)
	# 	pub1.publish(body)
	# 	T.join()

    # 		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

