#!/usr/bin/env python
import rospy
from copy import deepcopy
#import sys
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode 
import math
import time

msg = OverrideRCIn()
msg.channels[0] = 0
msg.channels[1] = 0
msg.channels[2] = 0
msg.channels[3] = 0
msg.channels[4] = 0
msg.channels[5] = 0
msg.channels[6] = 0
msg.channels[7] = 0

throttle = 2
steer = 0

#compass heading, latitude and longitude values
#pos_data=[heading,latitude,longitude]
pos_data = [0, 0, 0]

#starting point info
start_pos = [0, 0, 0]
start_heading = 'n'
stArt = True
hdg_set = False
gps_set = False
pos_set = False

#SEC0: Initialize parameters
x_dist = 5 #distance for all forward motion waypoints in meters
y_dist = 5 #distance toward 90 deg to left side to be covered in meters
MAX_SPEED = 1550
MIN_SPEED = 1510
SPEED = MAX_SPEED
max_angle = 350
MAX_TURN = 1500 + max_angle
MIN_TURN = 1500 - max_angle
STOP = 1500

Kp = abs(max_angle) / 180.0 #Proportional controller gain
u_turn = False #robot should not make a turn if false

#SEC1: Compute Next WayPoint
x_coord = ['n'] * 2 #Initialize forward waypoint to invalid value
y_coord = ['n'] * 2 #Initialize end coordinates to invalid value
earthR = 637100.0 #metres

#a. compute new lon and lat when given curr lon, lat and distance required
def new_waypoint(pos,d):
	theta,lon1,lat1 = pos
	delta = d / earthR

	theta = math.radians(theta)
	lat1 = math.radians(lat1)
	lon1 = math.radians(lon1)

	lat2 = math.asin(math.sin(lat1) * math.cos(delta) + math.cos(lat1) * math.sin(delta) * math.cos(theta))
	lon2 = lon1 + math.atan2(math.sin(theta) * math.sin(delta) * math.cos(lat1), math.cos(delta) - math.sin(lat1) * math.sin(lat2))
	lon2 = (lon2 + 3 * math.pi) % (2 * math.pi) - math.pi

	return [math.degrees(lon2),math.degrees(lat2)]

#b. compute distance when given two sets of lon and lat
def compute_d(pos1,pos2):
	lon1,lat1 = pos1
	lon2,lat2 = pos2

	latd = math.radians(lat2 - lat1)
	lond = math.radians(lon2 - lon1)

	lat1 = math.radians(lat1)
	lat2 = math.radians(lat2)

	a = math.sin(latd/2) * math.sin(latd/2) + math.sin(lond/2) * math.sin(lond/2) * math.cos(lat1) * math.cos(lat2)
	c = 2 * math.atan2(math.sqrt(a),math.sqrt(1-a))

	return earthR * c

#SEC2: Linear and steering controls
def uTurn(curr_pos,drd_hdg):
	global msg, u_turn, x_coord
	#if abs(drd_hdg - start_hdg) > 5:
	msg.channels[steer] = MIN_TURN
	#else:
		#msg.channels[steer] = MAX_TURN
	if abs(curr_pos[0] - drd_hdg) < 5:
		u_turn = False
		x_coord = new_waypoint(curr_pos,x_dist)
	#	if abs(drd_hdg - start_heading) < 5 and compute_d(pos_data[1:],y_coord) > compute_d(start_pos[1:],y_coord):
	#		print 'Destination Reached. Stoppin'
	#		msg.channels[steer] = 1500
	#		msg.channels[throttle] = 1500
	msg.channels[throttle] = SPEED

#I don't think I used this normalize function anywhere
def normalize_angle(theta):
	return 180 - theta
def sTop():
	global msg
	msg.channels[throttle]=1500
	msg.channels[steer] = 1500
	
def move_forward(hdg,drd_hdg):
	global msg
	msg.channels[throttle] = SPEED
	err = drd_hdg - hdg
	if err >= 180:
		err = err - 360
	elif err <= -180:
		err = err + 180

	u = Kp * err
	turn_amt = 1500 + u

	if turn_amt > MAX_TURN:
		turn_amt = MAX_TURN
	elif turn_amt < MIN_TURN:
		turn_amt = MIN_TURN
	msg.channels[steer] = turn_amt
t1 = time.time()
def explore():
	global x_coord,y_coord,start_pos,u_turn,start_heading,stArt,t1
	if stArt:
		#start_pos = deepcopy(pos_data)
		start_heading = start_pos[0]
		#compute end point
		y_coord = new_waypoint([start_pos[0]+90, start_pos[1],start_pos[2]],y_dist)
		stArt = False
		#compute waypoint ahead
		x_coord = new_waypoint(start_pos,x_dist)
		print 'start_pos = ',start_pos
		print 'y_coord = ',y_coord
		print 'x_coord = ',x_coord
	
	#if compute_d(pos_data[1:],x_coord) < 0.3 and not u_turn:
	if time.time() - t1 >= 3:
		#close to current waypoint
		start_pos[0] = (start_pos[0] + 180) % 360
		#u_turn = True
		sTop()
	else:
		move_forward(pos_data[0],start_pos[0])
	if u_turn:
		uTurn(pos_data,start_pos[0])
		t1 = time.time()
	#else:
	#move_forward(pos_data[0],start_pos[0])
cmd = 't'
def cmd_callback(data):
	global cmd
	cmd = data.data
	
       
def setHeading(data):
	global pos_data
	global hdg_set
        pos_data[0] = data.data
	hdg_set = True

def set_lat_lon(data):
	global pos_data
	global gps_set
        pos_data[1] = data.latitude
        pos_data[2] = data.longitude
	gps_set = True
  
Action = 'Teleoperated'
def listener():
    global Action
    global start_pos
    global pos_set
    sub = rospy.Subscriber('/my_cmd',String,cmd_callback)
    subhdg = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,setHeading)
    sub_lat_lon = rospy.Subscriber('/mavros/global_position/global',NavSatFix,set_lat_lon)
    pub = rospy.Publisher('/mavros/rc/override',OverrideRCIn, queue_size=10)
    action_pub = rospy.Publisher('/myAction',String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
		pub.publish(msg)
		
		if hdg_set and gps_set and not pos_set:
			start_pos = deepcopy(pos_data)
			#print 'pose now set'
			pos_set = True
		#print msg
		if pos_set and cmd=='e':
			explore()
			Action = 'explore'
		elif cmd==' ':
			sTop()
			Action = 'stop'
		elif cmd=='t':
			Action = 'Teleoperated'
		elif cmd=='q':
			return
		
		else:
			Action = 'Unrecognized'
		action_pub.publish(Action)
		rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")




if __name__=='__main__':
        node =     rospy.init_node('my_rover',anonymous=True)
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
    
