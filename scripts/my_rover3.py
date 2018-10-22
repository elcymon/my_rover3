#!/usr/bin/env python
import rospy
import roslib
from copy import deepcopy
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode 
from sensor_msgs.msg import Imu,MagneticField
from geometry_msgs.msg import Vector3Stamped
from tf.transformations import euler_from_quaternion
from mavros_msgs.srv import CommandBool
import math
import time
import angles

msg = OverrideRCIn()

msg1 = deepcopy(msg)

throttle = 2
steer = 0

#compass heading, latitude and longitude values

#starting point info
start_heading = 0
my_heading = 0
drd_heading = 0

count = 2
tx = 0
linear_dur = 15

stArt = True
hdg_set = False

#SEC0: Initialize parameters
MAX_SPEED = 1650
MIN_SPEED = 1350
REV_SPEED = 1350

SPEED = 1562 #initial speed set to stationary
drd_rps = 5 #desired rpm for the robot to move at. This is the setpoint for controller
vel_ctrl_effort = 0
vel_ctrl_effort_scale = 1.5

max_angle = 390
MAX_TURN = 1500 + max_angle
MIN_TURN = 1500 - max_angle
STOP = 1500
hdg_ctrl_effort = 0
hdg_ctrl_effort_scale= 3.9
u_turn = False #robot should not make a turn if false

#SEC1: Compute Next WayPoint

#a. compute new lon and lat when given curr lon, lat and distance required

#b. compute distance when given two sets of lon and lat

#SEC2: Linear and steering controls
def sTop():
	global msg
#	print type(msg)
	msg = OverrideRCIn()
	msg.channels[steer]=STOP
	msg.channels[throttle] = STOP
ST = True
tt = time.time()
turn_plus = 1500
def move_forward():
	global msg,SPEED,tt,ST,turn_plus
	if ST:
		tt = time.time()
		ST = False
	# if time.time() - tt >= 10:
	# 	tt = time.time()
	# 	SPEED = SPEED + 8
	
	msg = OverrideRCIn()
	robot_speed =  SPEED + vel_ctrl_effort_scale * vel_ctrl_effort

	if robot_speed > MAX_SPEED:
		robot_speed = MAX_SPEED

	if robot_speed < MIN_SPEED:
		robot_speed = MIN_SPEED
        
        
	msg.channels[throttle] = int(robot_speed)
	
	turn_amt = 1500 + hdg_ctrl_effort_scale*hdg_ctrl_effort
	
	if turn_amt > MAX_TURN:
		turn_amt = MAX_TURN
	elif turn_amt < MIN_TURN:
		turn_amt = MIN_TURN
	msg.channels[steer] = int(turn_amt)

t1 = time.time()
t0 = 0
hdg_setpoint = 0
hz = 100
turn_t = 15.0
hdg_ramp = (180.0 - 0.0) / (hz * turn_t) # make 180 degree turn in  turn_t seconds, broken into each iteration loop hz

def explore():
	global my_heading,start_heading,stArt,t1,tx,drd_heading,ST,t0,hdg_setpoint,hdg_ramp
	# f = True #just to disable all else, set to true if not needed
	if stArt:
		ST = True
		t1 = time.time()
		start_heading = my_heading
		drd_heading = start_heading # uncomment if controller gain is set
		hdg_setpoint = start_heading
		#drd_heading = 110.0 # when trying to set the gain of controller using only specific angle
		stArt = False
	
	# print msg.channels[throttle],'\t',rps,'\t',msg.channels[steer],'\t',my_heading,'\t',hdg_setpoint,'\t',hdg_ramp,'\t',drd_heading
	
	if time.time() - t1 >= linear_dur:
		drd_heading = (drd_heading + 180) % 360
		tx = tx + 1
		t1 = time.time()
	
	if abs(drd_heading - start_heading) < 1:
		hdg_ramp = - abs(hdg_ramp)
	else:
		hdg_ramp = abs(hdg_ramp)

	if abs(hdg_setpoint - drd_heading) < abs(hdg_ramp):
		hdg_setpoint = drd_heading
	else:
		hdg_setpoint = (hdg_setpoint + hdg_ramp) % 360
		t1 = time.time()

	if tx > count and False:
		print 'end'
		sTop()
		t1 = time.time()
	else:
		move_forward()
		
		
	
			

cmd = ' '
def cmd_callback(data):
	global cmd
	cmd = data.data
	
       
def setHeading(data):
	'''subscriber topic for compass'''
	global my_heading
	global hdg_set
	# my_heading = data.data
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
	if data.data > 100:
		hdg_ctrl_effort = 100
	elif data.data < -100:
		hdg_ctrl_effort = -100
	else:
		hdg_ctrl_effort = data.data

	if cmd !='e':
		hdg_ctrl_effort = 0


def vel_ctrl_callback(data):
	global vel_ctrl_effort
	if data.data > 100:
		vel_ctrl_effort = 100
	elif data.data < -100:
		vel_ctrl_effort = -100
	else:
		vel_ctrl_effort = data.data
roll = 0
pitch = 0
yaw = 0
def imu_callback(data):
	global roll,pitch,yaw
	# roll = 0
	# pitch = 0
	# yaw = 0
	
	quaternion = (data.orientation.x,
		      data.orientation.y,
		      data.orientation.z,
		      data.orientation.w)
	(roll,pitch,yaw) = euler_from_quaternion(quaternion)
	yaw = (math.degrees(yaw) + 180) % 360

raw_roll = 0
raw_pitch = 0
raw_yaw = 0
pre_yaw = 0
pre_yaw_set = True
def madgwick_imu_callback(data):
	global raw_roll,raw_pitch,raw_yaw,p_yaw_raw,my_heading,hdg_set,pre_yaw,pre_yaw_set
	# raw_roll = 0
	# raw_pitch = 0
	# raw_yaw = 0
	
	quaternion = (data.orientation.x,
		      data.orientation.y,
		      data.orientation.z,
		      data.orientation.w)
	(raw_roll,raw_pitch,raw_yaw) = euler_from_quaternion(quaternion)
	raw_yaw = (math.degrees(raw_yaw) + 180) % 360
	my_heading = raw_yaw
	if pre_yaw_set:
		pre_yaw_set = False
		pre_yaw = raw_yaw
	hdg_set = True
	
	

def callback_rps(data):
	global rps
	rps = data.data

imu_msg = Imu()

def callback_imudata(data):
	global imu_msg
	imu_msg = Imu()
	imu_msg = data

geo_msg = Vector3Stamped()
mag_msg = MagneticField()
def callback_mag(data):
	global mag_msg, geo_msg
	mag_msg = MagneticField()
	geo_msg.header = mag_msg.header

	mag_msg = data
	geo_msg.vector = mag_msg.magnetic_field

rps = 0 #initialize rps to 0 at start of the experiment

def serial_callback_rps(data):
	global rps
	rps = float(data.data)

def serial_callback_imu(data):
	global imu_msg, mag_msg, geo_msg
	acc_mag_gyr = [float(i) for i in data.data.split(',')]
	# acc_mag_gyr = [float(i) for i in data.data.split(',')[1:]]
	G = 9.81
	gyro_sensitivity = 0.00875 * math.pi / 180.0
	accelerometer_sensitivity = 0.000061 * G
	mag_sensitivity = 0.095  * math.pi / 180.0
	pos = [0,4,8]
	
	imu_msg = Imu()

	imu_msg.header.frame_id = 'imu'
	imu_msg.header.stamp = rospy.Time.now()

	imu_msg.linear_acceleration.x = acc_mag_gyr[0] 
	imu_msg.linear_acceleration.y = acc_mag_gyr[1] 
	imu_msg.linear_acceleration.z = acc_mag_gyr[2] 
	
	mag_msg = MagneticField()
	mag_msg.header.frame_id='imu'
	mag_msg.header.stamp = rospy.Time.now()
	geo_msg.header = mag_msg.header
	#value is in Gauss, multiply by 1e-4 to convert to Tesla
	mag_msg.magnetic_field.x = acc_mag_gyr[3] * 1e-4
	mag_msg.magnetic_field.y = acc_mag_gyr[4] * 1e-4
	mag_msg.magnetic_field.z = acc_mag_gyr[5] * 1e-4
	geo_msg.vector = mag_msg.magnetic_field

	imu_msg.angular_velocity.x = acc_mag_gyr[6] * math.pi / 180.0
	imu_msg.angular_velocity.y = acc_mag_gyr[7] * math.pi / 180.0
	imu_msg.angular_velocity.z = acc_mag_gyr[8] * math.pi / 180.0
	#print(data.data)
	for p in pos:
		imu_msg.linear_acceleration_covariance[p] = accelerometer_sensitivity * accelerometer_sensitivity
		imu_msg.angular_velocity_covariance[p] = gyro_sensitivity * gyro_sensitivity 
		mag_msg.magnetic_field_covariance[p] = mag_sensitivity * mag_sensitivity
gyro_drift = 0	
def listener():
	global Action
	global start_pos
	global pos_set
	global msg
	global t1
	global stArt
	global pre_yaw
	global gyro_drift
	global tx
	sub = rospy.Subscriber('/my_cmd',String,cmd_callback,queue_size=1)
	#subhdg = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,setHeading,queue_size=1)
	subimu_mavros = rospy.Subscriber('/mavros/imu/data',Imu,imu_callback,queue_size=1)
	sub_madgwick_imu = rospy.Subscriber('/imu/data',Imu,madgwick_imu_callback,queue_size=1)
	# sub_rpi_arduino_imu = rospy.Subscriber('/pub_imu',String,serial_callback_imu,queue_size=1)
	sub_rpi_arduino_rps = rospy.Subscriber('/pub_rps',String,serial_callback_rps,queue_size=1)
	submavrosimuraw = rospy.Subscriber('/mavros/imu/data_raw',Imu,callback_imudata,queue_size=1)
	submavrosmag = rospy.Subscriber('/mavros/imu/mag',MagneticField,callback_mag,queue_size=1)

	# pub_madgwick_imu = rospy.Publisher('/imu/data_raw',Imu, queue_size=1)
	# pub1_madgwick_mag = rospy.Publisher('/imu/mag',MagneticField, queue_size=1)
	# pub1_madgwick_geo = rospy.Publisher('/imu/magnetic_field',Vector3Stamped, queue_size=1)

	#    sub_lat_lon = rospy.Subscriber('/mavros/global_position/global',NavSatFix,set_lat_lon)
	pub = rospy.Publisher('/mavros/rc/override',OverrideRCIn, queue_size=1)
	action_pub = rospy.Publisher('/myAction',String, queue_size=1)
	sub_rps = rospy.Subscriber('/my_rover/rps',Float64,callback_rps,queue_size=1)
	vel_cmd_sub = rospy.Subscriber('/vel_cmd',OverrideRCIn,vel_cmd_callback,queue_size=1)

	#CONTROLLER PUBLISHERS AND SUBSCRIBERS
	hdg_ctrl_sub = rospy.Subscriber('/hdg/control_effort',Float64,hdg_ctrl_callback,queue_size=1)
	hdg_ctrl_pub = rospy.Publisher('/hdg/setpoint',Float64,queue_size=1)
	hdg_state_pub = rospy.Publisher('/hdg/state',Float64,queue_size=1)

	#VELOCITY CONTROLLER PUBLISHERS AND SUBSCRIBERS
	vel_ctrl_sub = rospy.Subscriber('/vel/control_effort',Float64,vel_ctrl_callback,queue_size=1)
	vel_ctrl_pub = rospy.Publisher('/vel/setpoint',Float64,queue_size=1)
	vel_state_pub = rospy.Publisher('/vel/state',Float64,queue_size=1)

	pub_data = rospy.Publisher('/data_collection',String,queue_size=1)
	
	rate = rospy.Rate(hz)
	# with open('/home/turtlebot/experiments/catkin_ws/results/'+time.strftime('%Y%m%d%H%M%S') + 'data.txt','a') as d, open('/home/turtlebot/experiments/catkin_ws/results/'+time.strftime('%Y%m%d%H%M%S') + 'teledata.txt', 'a') as t:
	# 	d.write('start session\n')
	# 	t.write('start session\n')
	# 	d.write(','.join(['throttle','rpm','control_effort','steering','drd_heading','my_heading','imu_yaw','imu_raw_yaw','time\n']))
	# 	t.write(','.join(['throttle','rpm','control_effort','steering','drd_heading','my_heading','imu_yaw','imu_raw_yaw','time\n']))
	if True:	
		header = ','.join(['time','throttle','rps','drd_rps','vel_control_effort','steering','my_heading','hdg_setpoint','drd_heading','hdg_ctrl_effort\n'])
		
		# pub_data.publish(header)
		#print header
		p_rpm = 0.0#previous rpm
		experiment_timer = time.time()
		set_experiment_timer = True
		while not rospy.is_shutdown():
			print msg.channels[throttle],'\t',rps,'\t',msg.channels[steer],'\t',my_heading,'\t',hdg_setpoint,'\t',hdg_ramp,'\t',drd_heading
	#		print yaw,'\t',raw_yaw,'\t',my_heading
			#rpm = geti2cData()#instantaneous rpm
			
			if cmd=='e':
				if hdg_set:
					if set_experiment_timer:
						set_experiment_timer = False
						experiment_timer = time.time()
					explore()
					j = ','.join(map(str,[time.time()-experiment_timer,msg.channels[throttle],rps,drd_rps,round(vel_ctrl_effort,2),msg.channels[steer],my_heading,hdg_setpoint,drd_heading,round(hdg_ctrl_effort,2)]))
					data = j + '\n'
					pub_data.publish(header+':'+data)
				else:
					print 'ERROR: Heading not set'
					
				Action = 'explore'
			elif cmd==' ':
				sTop()
				print 'resetting tx,t1 and stArt variables'
				set_experiment_timer = True
				tx = 0
				t1 = time.time()
				stArt = True
				Action = 'stop'
			elif cmd=='t':
				stArt = True
				Action = 'Teleoperated'
				msg = deepcopy(msg1)
				j2 = ','.join(map(str,[time.time()-experiment_timer,msg.channels[throttle],rps,drd_rps,round(vel_ctrl_effort,2),msg.channels[steer],my_heading,hdg_setpoint,drd_heading,round(hdg_ctrl_effort,2)]))
				# t.write(j2 + '\n')

			elif cmd=='q':
				#rospy.on_shutdown('Q pressed')
				Action = 'Quit'
				break
			else:
				Action = 'Unrecognized'
				print 'ERROR: Unrecognized command:',cmd
		
	#		print msg,msg1
			
			#explore()
			action_pub.publish(Action)
			#print msg
			pub.publish(msg)
			
			hdg_ctrl_pub.publish(hdg_setpoint)#publish heading setpoint #drd_heading desired heading
			hdg_state_pub.publish(my_heading)#publish current heading
			
			
			vel_ctrl_pub.publish(drd_rps)#publish desired velocity in rpm
			vel_state_pub.publish(rps) #publish current velocity in rpm

			# pub_madgwick_imu.publish(imu_msg)
			# pub1_madgwick_mag.publish(mag_msg)
			# pub1_madgwick_geo.publish(geo_msg)

		
#data collection stage
	
			rate.sleep()
                #rospy.spinOnce()
    #except KeyboardInterrupt:
	print("Shutting down")

start_listener = False
def callback_start(data):
	global start_listener
	start_listener = data.data


if __name__=='__main__':
#	global msg
	sub_start_listener = rospy.Subscriber('/my_rover/start_listener',Bool,callback_start)
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
	
	rospy.wait_for_service('/mavros/cmd/arming')
	try:
		armingService = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
		x  = armingService(True)
		print 'arming result is',x
	except rospy.ServiceException, e:
		print "Service arm call failed:%s"%e
	
	rospy.wait_for_service('/mavros/set_mode')
	change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
	answer = change_mode(custom_mode='manual')
    
	print('starting sim')
	start_listener = True
	while not start_listener:
		continue
	
	print('answer = ',answer)
	print('start_listener = ',start_listener)
	if start_listener:
		try:
			listener()
		except rospy.ROSInterruptException:
			print('Something Went wrong')
		pass
    
