#!/usr/bin/env python
import rospy
import roslib
from copy import deepcopy
#import sys
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math
import time

import smbus #needed for i2c communication
#import pigpio
bus = smbus.SMBus(1)
address = 0x04
def i2cData():
	try:
		data = bus.read_i2c_block_data(address,0)
	except IOError:
		data = 'invalid'
	return data
def geti2cData():
	data_i2c = i2cData()
	
	ss = 'invalid'
	if data_i2c == 'invalid':
		ss = 'invalid'
	else:
		d = [x for x in data_i2c if x==46 or (x >=48 and x <=57)]
		s1 = map(chr,d)
		ss = ''.join(s1)
	return ss


msg = OverrideRCIn()

msg1 = deepcopy(msg)

throttle = 2
steer = 0

#compass heading, latitude and longitude values
#pos_data=[heading,latitude,longitude]
#pos_data = [0, 0, 0]

#starting point info
#start_pos = [0, 0, 0]
start_heading = 0
my_heading = 0
drd_heading = 0

count = 2
tx = 0
linear_dur = 10

stArt = True
hdg_set = False
#gps_set = False
#pos_set = False

#SEC0: Initialize parameters
x_dist = 5 #distance for all forward motion waypoints in meters
y_dist = 5 #distance toward 90 deg to left side to be covered in meters
MAX_SPEED = 1600
MIN_SPEED = 1400
REV_SPEED = 1400

SPEED = MIN_SPEED #initial speed set to stationary
drd_rpm = 21.0 #desired rpm for the robot to move at. This is the setpoint for controller
vel_ctrl_effort = 0
vel_ctrl_effort_scale = 2

max_angle = 390
MAX_TURN = 1500 + max_angle
MIN_TURN = 1500 - max_angle
STOP = 1500
hdg_ctrl_effort = 0
hdg_ctrl_effort_scale=10
#Kp = abs(max_angle) / 180.0 #Proportional controller gain
u_turn = False #robot should not make a turn if false

#SEC1: Compute Next WayPoint
#x_coord = ['n'] * 2 #Initialize forward waypoint to invalid value
#y_coord = ['n'] * 2 #Initialize end coordinates to invalid value
#earthR = 637100.0 #metres

#a. compute new lon and lat when given curr lon, lat and distance required

#b. compute distance when given two sets of lon and lat

#SEC2: Linear and steering controls
def uTurn(drd_hdg):
	global msg, u_turn#, x_coord
	msg = OverrideRCIn()
	
	if abs(drd_hdg - start_heading) > 5:
		msg.channels[steer] = MIN_TURN
	else:
		msg.channels[steer] = MAX_TURN
	if abs(my_heading - drd_hdg) < 5:
		print 'my_heading:',my_heading,'drd_hdg',drd_hdg
		u_turn = False
	
	msg.channels[throttle] = SPEED

def sTop():
	global msg
#	print type(msg)
	msg = OverrideRCIn()
	msg.channels[steer]=STOP
	msg.channels[throttle] = STOP
ST = True
tt = time.time()
def move_forward():
	global msg,SPEED,tt,ST
	if ST:
		tt = time.time()
		ST = False
	if time.time() - tt >= 10:
		tt = time.time()
		SPEED = SPEED + 8
	
	msg = OverrideRCIn()
	robot_speed =  1500#SPEED #+ vel_ctrl_effort_scale * vel_ctrl_effort

	if robot_speed > MAX_SPEED:
                robot_speed = MAX_SPEED
        if robot_speed < MIN_SPEED:
                robot_speed = MIN_SPEED
        
        
        msg.channels[throttle] = robot_speed
	
	turn_amt = 1500 #+ hdg_ctrl_effort_scale*hdg_ctrl_effort

	if turn_amt > MAX_TURN:
		turn_amt = MAX_TURN
	elif turn_amt < MIN_TURN:
		turn_amt = MIN_TURN
	msg.channels[steer] = int(turn_amt)
t1 = time.time()
t0 = 0
def explore():
	global my_heading,u_turn,start_heading,stArt,t1,tx,drd_heading,ST,t0
	f = False #just to disable all else, set to true if not needed
	if stArt:
		ST = True
		t1 = time.time()
		start_heading = my_heading
		#drd_heading = start_heading # uncomment if controller gain is set
		drd_heading = 110.0 # when trying to set the gain of controller using only specific angle
		stArt = False
	t0 += time.time() - t1
	t1 = time.time()
	print t0 % 65, yaw,'\t',raw_yaw,'\t',my_heading
	
	if tx >count and f:
		print 'end'
		sTop()
		t1 = time.time()
	elif time.time() - t1 >= linear_dur and f:
		drd_heading = (drd_heading + 180) % 360
		u_turn = True
		sTop()
		tx = tx + 1
		t1 = time.time()
		print 'turn true',start_heading
	elif u_turn and f:
		uTurn(drd_heading)
		t1 = time.time()
		print 'turning'
	else:
		move_forward()
	#	print tx,'straight'
		
		
	
			

cmd = ' '
def cmd_callback(data):
	global cmd
	cmd = data.data
	
       
def setHeading(data):
	global my_heading
	global hdg_set
        my_heading = data.data
	hdg_set = True


def vel_cmd_callback(data):
	global msg1
	msg1 = OverrideRCIn()

	cmd_speed = data.channels[throttle]
	cmd_steer = data.channels[steer]

	
	if cmd_speed > MAX_SPEED:
		cmd_speed = MAX_SPEED
	
	if cmd_speed < REV_SPEED:
		cmd_speed = REV_SPEED
	
	if cmd_steer > MAX_TURN:
		cmd_steer = MAX_TURN
	
	if cmd_steer < MIN_TURN:
		cmd_steer = MIN_TURN

	msg1.channels[throttle] = cmd_speed
	msg1.channels[steer] = cmd_steer
  
Action = 'Teleoperated'
def hdg_ctrl_callback(data):
        global hdg_ctrl_effort
        if cmd=='e':
	        hdg_ctrl_effort = data.data
	else:
		hdg_ctrl_effort = 0
def vel_ctrl_callback(data):
        global vel_ctrl_effort
        vel_ctrl_effort = data.data
roll = 0
pitch = 0
yaw = 0

def imu_callback(data):
	global roll,pitch,yaw
	roll = 0
	pitch = 0
	yaw = 0
	
	quaternion = (data.orientation.x,
		      data.orientation.y,
		      data.orientation.z,
		      data.orientation.w)
	(roll,pitch,yaw) = euler_from_quaternion(quaternion)
	yaw = (math.degrees(yaw) + 180) % 360

raw_roll = 0
raw_pitch = 0
raw_yaw = 0
def imu_raw_callback(data):
	global raw_roll,raw_pitch,raw_yaw
	raw_roll = 0
	raw_pitch = 0
	raw_yaw = 0
	
	quaternion = (data.orientation.x,
		      data.orientation.y,
		      data.orientation.z,
		      data.orientation.w)
	(raw_roll,raw_pitch,raw_yaw) = euler_from_quaternion(quaternion)
	raw_yaw = (math.degrees(raw_yaw) + 180) % 360
def listener():
    global Action
    global start_pos
    global pos_set
    global msg
    global t1
    global stArt
    
    sub = rospy.Subscriber('/my_cmd',String,cmd_callback,queue_size=1)
    subhdg = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,setHeading,queue_size=1)
    subimu = rospy.Subscriber('/mavros/imu/data',Imu,imu_callback,queue_size=1)
    subimu_raw = rospy.Subscriber('/mavros/imu/data_raw',Imu,imu_raw_callback,queue_size=1)
#    sub_lat_lon = rospy.Subscriber('/mavros/global_position/global',NavSatFix,set_lat_lon)
    pub = rospy.Publisher('/mavros/rc/override',OverrideRCIn, queue_size=1)
    action_pub = rospy.Publisher('/myAction',String, queue_size=1)
    
    vel_cmd_sub = rospy.Subscriber('/vel_cmd',OverrideRCIn,vel_cmd_callback,queue_size=1)

#CONTROLLER PUBLISHERS AND SUBSCRIBERS
    hdg_ctrl_sub = rospy.Subscriber('/hdg/control_effort',Float64,hdg_ctrl_callback,queue_size=1)
    hdg_ctrl_pub = rospy.Publisher('/hdg/setpoint',Float64,queue_size=1)
    hdg_state_pub = rospy.Publisher('hdg/state',Float64,queue_size=1)

#VELOCITY CONTROLLER PUBLISHERS AND SUBSCRIBERS
    vel_ctrl_sub = rospy.Subscriber('/vel/control_effort',Float64,vel_ctrl_callback,queue_size=1)
    vel_ctrl_pub = rospy.Publisher('/vel/setpoint',Float64,queue_size=1)
    vel_state_pub = rospy.Publisher('/vel/state',Float64,queue_size=1)
    
    rate = rospy.Rate(10)
    with open('/home/erle/ws/data.txt','a+') as d, open('/home/erle/ws/teledata.txt', 'a+') as t:
    	d.write('start session\n')
    	t.write('start session\n')
    	d.write(','.join(['throttle','rpm','control_effort','steering','drd_heading','my_heading','imu_yaw','imu_raw_yaw','time\n']))
    	t.write(','.join(['throttle','rpm','control_effort','steering','drd_heading','my_heading','imu_yaw','imu_raw_yaw','time\n']))
    	
    	p_rpm = 0.0#previous rpm
    	while not rospy.is_shutdown():
		
#		print yaw,'\t',raw_yaw,'\t',my_heading
		rpm = geti2cData()#instantaneous rpm
		
		if cmd=='e':
			if hdg_set:
				explore()
				j = ','.join(map(str,[msg.channels[throttle],rpm,round(hdg_ctrl_effort,2),msg.channels[steer],drd_heading,my_heading,yaw,raw_yaw,time.time()]))
				d.write(j + '\n')
			else:
				print 'ERROR: Heading not set'
				
			Action = 'explore'
		elif cmd==' ':
			sTop()
			Action = 'stop'
		elif cmd=='t':
			stArt = True
			Action = 'Teleoperated'
			msg = deepcopy(msg1)
			j2 = ','.join(map(str,[msg.channels[throttle],rpm,round(hdg_ctrl_effort,2),msg.channels[steer],drd_heading,my_heading,yaw,raw_yaw,time.time()]))
			t.write(j2 + '\n')

		elif cmd=='q':
			#rospy.on_shutdown('Q pressed')
			Action = 'Quit'
			break;
		else:
			Action = 'Unrecognized'
			print 'ERROR: Unrecognized command:',cmd
	
#		print msg,msg1
		
		#explore()
		action_pub.publish(Action)
		#print msg
		pub.publish(msg)
		hdg_ctrl_pub.publish(drd_heading)#publish desired heading
		hdg_state_pub.publish(my_heading)#publish current heading
		
		if rpm =='invalid':
			rpm = p_rpm
		else:
			rpm = float(rpm)
			rpm = rpm * 100.0 / 72.0
			if SPEED < 1500:
				rpm = -1 * rpm
		
		p_rpm = rpm
		
		vel_ctrl_pub.publish(drd_rpm)#publish desired velocity in rpm
		vel_state_pub.publish(p_rpm) #publish current velocity in rpm

		
#data collection stage
	
		rate.sleep()
                #rospy.spinOnce()
    #except KeyboardInterrupt:
    print("Shutting down")




if __name__=='__main__':
#	global msg
	msg = OverrideRCIn()
	msg.channels[0] = 0
	msg.channels[1] = 0
	msg.channels[2] = 0
	msg.channels[3] = 0
	msg.channels[4] = 0
	msg.channels[5] = 0
	msg.channels[6] = 0
	msg.channels[7] = 0

        node =     rospy.init_node('my_rover',anonymous=True)
        print('starting sim')
        rospy.wait_for_service('/mavros/set_mode')
        change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
        answer = change_mode(custom_mode='manual')
        print(answer)
        if 'True' in str(answer):
          try:
            listener()
          except rospy.ROSInterruptException:
            print('Something Went wrong')
            pass
    
