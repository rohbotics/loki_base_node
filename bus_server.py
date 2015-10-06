#!/usr/bin/env python

# Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

"""

After some discussions with an ex-Willow Garage person (Tony Pratkanis),
it became clear that we were not collecting accurate enough time stamps
from our sonar sensors.  The purpose of the code is to obtain and manage
better sensor timestamps.

In short, we are using the sonar sensors as a poor mans Lidar.  This
is because 16 sonars costs approximately $16, whereas a Lidar costs
much more than that.  The sonars do not provide enough information to
run a SLAM algorithm, but they do provide enough information for local
obstacle avoidance.

The ROS node that takes sensor data like Lidar and Sonar and
converts into a local cost map is called `navigation_layers`.
We are using the `range_sensor_layer` plugin for `navigation_layers`
because we are using sonars.  The `navigation_layers` package takes
the sonar information and generates a local cost map that can be used
by the robot to avoid running into things.

Since sonars are significantly slower than a Lidar, there is much more
sensor skew going on as the robot moves.  The `navigation_layers`
package uses the range sensor timestamps, to interpolate where the
platform was when the sensor was read.  This helps deal with sensor
skew and provides a more accurate local cost map.

Unfortunately, the sonar timestamps captured by the ROS Arduino Bridge
are very inaccurate.  This results in a very messy local cost map.
After a looking at what needed to be done and the current state of
the ROS Arduino Bridge code (which is already severely hacked code),
we came to the conclusion that it was time to write some new code
from scratch.

The primary command that is used in the code below is the "sensor queue
read" command which is implemented a single letter "q":

        q

The command returns the following:

        {dt} {sensor1} {sensor2} ....

where:

* {dt} is the number of microseconds elapsed since the last "q" command, and
* {sensorI} is a sensor reading.

Each sensor reading is a tripple of number that has the following form:

        {id}:{dt}:{value}

where:

* {id} is the sensor id (e.g. 0, 7, 17),
* {dt} is the microsecond time offset from current time (usually <=0, and
* {value} is the sensor value.

Since we generate a stream of "q" commands at 50 times a second and the sonars
are maintaining microsecond level timing, the sonar times are quite accurate
with relatively low latency.

It should be mentioned, that the "q" command is used to read the encoders
as well.  This results in better odometry.

"""

# Put all of the imports into alphabetical order:
from geometry_msgs.msg import Quaternion, Pose, Twist
from math import cos, degrees, pi as PI, radians, sin
from nav_msgs.msg import Odometry
import rospy
import os
from sensor_msgs.msg import Range
from serial import *
import sys
from tf.broadcaster import TransformBroadcaster
import thread
import time
import traceback

def main():
    bus_server = Bus_Server()
    bus_server.poll_loop()

class Bus_Server:
    """ *Bus_Server*: The Bus_Server encapsulates all the of the code
	needed to talk to the robot over a serial connection and
	publish/subscribe all requisite ROS topics for ROS.
    """

    def __init__(self):
	""" *Bus_Server*: Initialize the bus server.
	"""

	# Initialize the *logger*:
	self.logger_ = logger = Logger(True)
	logger.info("Starting bus server...")

	# Create the ROS initializion node:
	rospy.init_node("bus_server", log_level=rospy.DEBUG)

	# Clean up when terminating the node:
	rospy.on_shutdown(self.shut_down)

	# Grab some parameter values from the ROS parameter server:
	self.base_frame_ = base_frame = \
	  rospy.get_param("~base_frame", 'base_link')
	self.baud_rate_ = int(rospy.get_param("~baud", 57600))
	self.poll_rate_ = rospy.get_param("~poll_rate", 25)
	self.port_name_ = port_name = rospy.get_param("~port", "")
	self.timeout_ = rospy.get_param("~timeout", 0.5)

	# Probe for standard ports:
	if port_name == "":
	    if os.path.exists("/dev/ttyACM0"):
		port_name = "/dev/ttyACM0"
	    elif os.path.exists("/dev/ttyUSB0"):
		port_name = "/dev/ttyUSB0"
	    self.port_name_ = port_name

	# Grab some parameters for dead reckoning:
	self.encoder_resolution_ = encoder_resolution = \
	  int(rospy.get_param("~encoder_resolution", 1000))
	self.gear_reduction_ = gear_reduction = \
	  float(rospy.get_param("~gear_reduction", 1.0))
	self.pid_rate_ = pid_rate = \
	  rospy.get_param("~pid_rate", 50)
	self.wheel_diameter_ = wheel_diameter = \
	  float(rospy.get_param("~wheel_diameter", .100))
	self.wheel_spread_ = wheel_spread = \
	  float(rospy.get_param("~wheel_spread", .200))

	# Compute *ticks_per_meter*:
	self.ticks_per_meter_ = ticks_per_meter = \
	  float(encoder_resolution) * gear_reduction  / (wheel_diameter * PI)

	# Set up *Dead_Reckon* object:
	self.dead_reckon_ = dead_reckon = Dead_Reckon(base_frame = base_frame,
	  wheel_diameter = wheel_diameter, wheel_spread = wheel_spread,
	  encoder_resolution = encoder_resolution,
	  gear_reduction = gear_reduction, ticks_per_meter = ticks_per_meter)

	# Open the serial *connection* before setting up subscription to Twist:
	self.connection_ = connection = \
	  Connection(logger, port_name, 115200, self.timeout_)

	# Set up command velocity pulisher (do before *cmd_vel_callback*)
        # is setup.:
	self.cmd_vel_ = Twist()
	self.cmd_vel_pub_ = rospy.Publisher("cmd_vel", Twist, queue_size=5)

	# Subscribe to *cmd_vel* topic:
	self.cmd_vel_enabled_ = True
	rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

	# Overall loop rate: should be faster than fastest sensor rate
	self.rate_ = rospy.Rate(self.poll_rate_)

	# Slurp in the PID parameters:
	Kp = rospy.get_param("~Kp", 20)
	Kd = rospy.get_param("~Kd", 3)
	Ki = rospy.get_param("~Ki", 86)
	Ko = rospy.get_param("~Ko", 300)
	Ci = rospy.get_param("~Ci", 100)

	# Send PID initialization command:
	command = "u {0} {1} {2} {3} {4}".format(Kp, Kd, Ki, Ko, Ci)
	result = connection.execute(command).strip()
	logger.info("Command '{0}' returned '{1}'".format(command, result))

	# Create the *left_encoder* and *right_encoder* sensors:
	self.left_encoder_ = left_encoder = Encoder_Sensor(dead_reckon, True)
	self.right_encoder_ = right_encoder = Encoder_Sensor(dead_reckon, False)
	self.sensors_ = sensors = [left_encoder, right_encoder]

	# Create any remaining sensor from the parameters:
	sensor_params = rospy.get_param("~sensors", dict({}))
	for name, params in sensor_params.iteritems():
	    #logger.info("{0}: {1}".format(name, str(params)))
	    # The *pop**() method modifies *params*, so we make a copy first:
	    params = params.copy()

	    # Grab *sensor_id*, *frame_id*, and *sensor_type* from *params*:
	    sensor_id = params.pop("id", -1)
	    frame_id = params.pop('frame_id', self.base_frame_)
	    sensor_type = params.pop("type", "None")
            
	    # Dispatch on *sensor_type*:
	    sensor = None
	    if sensor_type == "Sonar":
		# Grab fields:
		field_of_view = params.pop("field_of_view", .43632347)
		min_range = params.pop("min_range", 0.02)
		max_range = params.pop("max_range", 100.00)
		class_name = params.pop("class", "Unknown")
		left_id = params.pop("left_id", "Unknown")
		right_id = params.pop("right_id", "Unknown")

		# We have sonar, so grab the additional fields and create
		# the *Sonar* object:
		sensor = Sonar_Sensor(name,
		  Sensor.SONAR_TYPE, sensor_id, frame_id,
		  field_of_view = field_of_view,
		  min_range = min_range,
		  max_range = max_range)

		sonar_class = -1
		if class_name.lower() == "off":
		    sonar_class = 0	# From Sonar.h
		elif class_name.lower() == "back":
		    sonar_class = 1	# From Sonar.h
		elif class_name.lower() == "front":
		    sonar_class = 2	# From Sonar.h
		elif class_name.lower() == "side":
		    sonar_class = 3	# From Sonar.h
		else:
		    logger.warn(
		      "Sonar class '{0}' not recognized".format(class_name))

		if sonar_class >= 0:
		    command = "s {0} {1} {2} {3}".format(sensor_id,
		      sonar_class, left_id, right_id).strip()
		    result = connection.execute(command).strip()
		    logger.info("Sent '{0}' and got '{1}'".
		      format(command, result))

		# Insert *sensor* into *sensors* infilling with empty slots
		# with *None*:
		if sensor_id >=0:
		    assert isinstance(sensor, Sensor)
		    while (len(sensors) <= sensor_id):
			sensors.append(None)
		    sensors[sensor_id] = sensor
	    else:
		logger.warn(
		  "Unrecognized sensor type '{0}'".format(sensor_type))

	# List all of the *sensors* in numerical order:
	for sensor in sensors:
	    if sensor != None:
		logger.info("[{0}]: {1}".format(sensor.sensor_id, sensor))

    def cmd_vel_callback(self, request):
	""" *Bus_Server*: This is routine is called each time a cmd_vel
	    (e.g. Twist) message is received.  The twist information is
	    passed in via the *request* argument.
	"""

	# Grab some additional values from *self*:
	cmd_vel_enabled = self.cmd_vel_enabled_
	connection = self.connection_
	gear_reduction = self.gear_reduction_
	logger = self.logger_
	pid_rate = self.pid_rate_
	ticks_per_meter = self.ticks_per_meter_
	wheel_spread = self.wheel_spread_

	# Only process a Twist message if *cmd_vel_enabled* is set:
	if cmd_vel_enabled:
	    # Extract the *x* and *theta* from *request*:
	    x = request.linear.x		# m/s
	    theta = request.angular.z	# rad/s

	    # Log the cmd_vel message information:
	    logger.info("CmdVel: x={0}m/s th={1}deg/s".
	      format(x, theta * 180.0/PI))

	    if x == 0.0:
		# Turn in place:
		right = theta * wheel_spread * gear_reduction / 2.0
		left = -right
	    elif theta == 0:
		# Pure forward/backward motion:
		left = right = x
	    else:
		# Rotation about a point in space:
		delta = theta * wheel_spread * gear_reduction / 2.0
		left =  x - delta
		right = x + delta
            
	    # Send a command the causes the motors to spin:
	    request_left = int(left * ticks_per_meter / pid_rate)
	    request_right = int(right * ticks_per_meter / pid_rate)
	    connection.execute("m {0} {1}".format(request_left, request_right))

    def poll_loop(self):
	""" *Bus_Server*: Perform polling for the *Bus_Server* object.
	"""

	# Grab values from *self*:
	base_frame = self.base_frame_
	connection = self.connection_
	dead_reckon = self.dead_reckon_
	logger = self.logger_
	rate = self.rate_
	sensors = self.sensors_
	poll_rate = self.poll_rate_

	# Define some types and routines for Time management and manipulation:
	Duration = rospy.Duration
	Time = rospy.Time
	now_routine = rospy.Time.now
	next_poll_time = now_routine()

	# Compute *delta_poll_time* as the reciprical of *poll_rate*:
	delta_poll_time = rospy.Duration(1.0 / float(poll_rate))

	# Start polling the sensors and base controller
	robot_base_time = None
	while not rospy.is_shutdown():
	    # Figure out if it is time to poll the sensor queue:
	    now = now_routine()
	    if now >= next_poll_time:
		# Update *next_poll_time*:
		next_poll_time += delta_poll_time

		# Grab the next back of sensor values from the sensor queue:
		poll_values = connection.execute("q").split()
		print("Poll: {0}".format(poll_values))

		# The first value in *poll_values* is the number of
		# microseconds that have elapsed on the robot since
		# the last time.  We need to add that to *robot_base_time*:
		if len(poll_values) == 0:
		    logger.warn("Missing time delta from 'q' command")
		else:
		    # We have the microseonds value in *poll_values[0]*:
		    #if robot_base_time == None:
		    if True:
			# This is the first time, so we just use *now()*:
			robot_base_time = now_routine()
		    else:
			# Convert from integer microseconds into float and
			# add it to *robot_base_time*:
			try:
			    robot_base_time += \
			      Duration(float(poll_values[0]) / 1000000.0)
			except:
			    # Something went wrong, so reset *robot_base_time*:
			    robot_base_time = now_routine()
		    #assert isinstance(robot_base_time, Time)

		    #print("robot_base_time={0:d}.{1:09d}".
		    # format(robot_base_time.secs,robot_base_time.nsecs))

		    # Now process any remaining *poll_values*:
		    #print("poll_values=", poll_values)
		    for poll_value in poll_values[1:]:
			#print("poll_value=", poll_value)
			id_time_value = poll_value.split(':')
			sensor_id = int(id_time_value[0])
			#print("sensor_id={0}".format(sensor_id))
			time = robot_base_time + \
			  Duration(float(id_time_value[1]) / 1000000.0)
			#print("time={0:d}.{1:09d}".
			#  format(time.secs, time.nsecs))
			value = int(id_time_value[2])
			#print("value={0}".format(value))
			sensor = sensors[sensor_id]
			sensor.value_set(value, time)

		    # Deal with any dead reckoning publishing:
		    dead_reckon.publish(now)
            
	    # Sleep until next time:
	    rate.sleep()
    
    def shut_down(self):
	""" *Bus_Server*: Shut down the bus_server.
	"""

	# Cause future cmd_vel messages to be ignored::
	self.cmd_vel_enabled_ = False

	# Grab some values from *self*:
	connection = self.connection_
	logger = self.logger_

	# Stop the robot
	command = "m 0 0"
	result = connection.execute(command).strip()
	logger.info("Sent '{0}' and received '{1}'".format(command, result))
	logger.info("Shutting down Bus Server node...")

	# Close the serial connection:
	self.connection_.close()

class Connection:
    """ *Connection*: Mangage the serial port connection.
    """

    def __init__(self, logger, port_name, baud_rate, timeout):
	""" *Connection*: Initialize a connection using *port_name*,
	    *baud_rate* and *timeout*.
	"""

	# Verify argument types:
	assert isinstance(logger, Logger)
	assert isinstance(port_name, str)
	assert isinstance(baud_rate, int)
	assert isinstance(timeout, int) or isinstance(timeout, float)

	# Load up *self*:
	self.baud_rate_ = baud_rate
	self.logger_ = logger
        self.mutex_ = thread.allocate_lock()
	self.port_name_ = port_name
	self.timeout_ = timeout

	# Open the connection:
	logger.info("Connecting to serial port '{0}' ...".format(port_name))
	try:
	    self.port_ = port = Serial(port=port_name, baudrate=baud_rate,
	      bytesize=EIGHTBITS, parity=PARITY_NONE, stopbits=STOPBITS_ONE,
	      timeout=timeout, writeTimeout=timeout, xonxoff=False,
	      rtscts=False, dsrdtr=False)
	    if not port.isOpen():
	      logger.fatal("Serial port '{0}' is not open".format(port_name))

	    # Read the baud rate to make sure we are connected:
	    # FIXME: We should a a firmware version number command!!!
	    baud = 0
	    for count in range(3):
		# Give the firmware some time to wake-up:
		time.sleep(1)
		baud = self.baud_get()
		if baud == baud_rate:
		    break
	    if baud != baud_rate:
		raise SerialException
	except SerialException:
	    # If we get here, we die:
	    logger.warn("Serial Exception:")
	    logger.warn(sys.exc_info())
	    logger.warn("Traceback follows:")
	    logger.warn(traceback.format_exc())
	    logger.fatal("Cannot connect to robot!")

	# Log that we established a connection:
	logger.info("Serial connection at {0} baud".format(baud_rate))

    def baud_get(self):
	""" *Connection*: Return the baud rate from the connected platform.
	"""

	result = self.execute("b");
	try:
	    baud_rate = int(result)
	except:
	    baud_rate = 0;
	    self.logger_.warn("Connection.baud_get: Got '{0}', not baud_rate".
	      format(result))
	return baud_rate

    def close(self): 
	""" *Connection*: Close the connection:
	"""

	self.port_.close() 

    def execute(self, command):
	""" *Connection*: Thread safe execution of "command" on the Arduino
	    returning a single integer value.
	"""

	#print("=>Connection.execute()")
	#rospy.loginfo("Send command '{0}' to arduino".format(cmd))

	# Make sure we are the only thread using the connection:
	self.mutex_.acquire()
        
	port = self.port_
	try:
	    # Make sure thare is no input left over from a previous command:
	    port.flushInput()

	    # Send *command* followed by a carriage return:
	    port.write(command)
	    port.write("\r")
	    port.flush()

	    value = port.readline()
	except SerialException:
	    print("Serial Exception Occurred")

	# Release the lock for the next thread:
	self.mutex_.release()

	return value

class Dead_Reckon:
    """ *Dead_Reckon*: This class uses a pair of *Encoder* object to keep
	track of the robot base location and orientation.
    """

    def __init__(self, base_frame, wheel_diameter, wheel_spread,
      encoder_resolution, gear_reduction, ticks_per_meter):
	""" *Dead_Reckon*: Initialize an *Odometry* object to contain
	    *base_frame*, 
	"""

	# Verify argument types:
	assert isinstance(base_frame, str)
	assert isinstance(encoder_resolution, int)
	assert isinstance(gear_reduction, float)
	assert isinstance(ticks_per_meter, float)
	assert isinstance(wheel_diameter, float)
	assert isinstance(wheel_spread, float)

	# Get the starting time *now*:
	now = rospy.Time.now()

	# Load up *self*:
	self.base_frame_ = base_frame
	self.covariance_ = [
	  0.2,  0.0,  0.0,  0.0,  0.0,  0.0,
	  0.0,  0.2,  0.0,  0.0,  0.0,  0.0,
	  0.0,  0.0,  0.2,  0.0,  0.0,  0.0,
	  0.0,  0.0,  0.0,  0.2,  0.0,  0.0,
	  0.0,  0.0,  0.0,  0.0,  0.2,  0.0,
	  0.0,  0.0,  0.0,  0.0,  0.0,  0.2 ]
	self.encoder_resolution_ = encoder_resolution
	self.gear_reduction_ = gear_reduction
	self.left_encoder_ = 0
	self.right_encoder_ = 0
	self.now_ = now
	self.odom_ = Odometry()
	self.odom_pub_ = rospy.Publisher('odom', Odometry, queue_size=5)
	self.odom_broadcaster_ = TransformBroadcaster()
	self.previous_left_encoder_ = 0
	self.previous_now_ = now
	self.previous_right_encoder_ = 0
	self.quaternion_ = Quaternion()
	self.th_ = 0.0	# Theta is the robot bearing
	self.ticks_per_meter_ = ticks_per_meter
	self.wheel_diameter_ = wheel_diameter
	self.wheel_spread_ = wheel_spread
	self.x_ = 0.0
	self.y_ = 0.0

    def left_encoder_set(self, value, time):
	""" *Dead_Reckon*: Set the left encoder to *value* and *time*.
	"""

	# Verify argument types:
	assert isinstance(value, int)
	#assert isinstance(value, rospy.Time)

	# Load up *self*:
	self.left_encoder_ += value
	self.left_encoder_time_ = time

    def publish(self, now):
	""" *Dead_Reckon*: Publish the odometry topic if needed.
	"""

	# Grab some values out of *self* (alphabetical order):
	base_frame = self.base_frame_
	covariance = self.covariance_
	left_encoder = self.left_encoder_
	odom = self.odom_
	odom_broadcaster = self.odom_broadcaster_
	odom_pub = self.odom_pub_
	previous_left_encoder = self.previous_left_encoder_
	previous_now = self.previous_now_
	previous_right_encoder = self.previous_right_encoder_
	quaternion = self.quaternion_
	right_encoder = self.right_encoder_
	th = self.th_
	ticks_per_meter = self.ticks_per_meter_
	wheel_spread = self.wheel_spread_
	x = self.x_
	y = self.y_

	# Compute *dt*:
	delta_time = now - previous_now
	dt = delta_time.to_sec()
	
	# Calculate odometry intermediate values:
	dleft = float(left_encoder - previous_left_encoder) / ticks_per_meter
	dright = float(right_encoder - previous_right_encoder) / ticks_per_meter
	dxy_average = (dright + dleft) / 2.0
	dth = (dright - dleft) / wheel_spread
	vxy = dxy_average / dt
	vth = dth / dt
	    
	# Update *x*, *y*, and *th* (theta):
	if (dxy_average != 0.0):
	    dx =  dxy_average * cos(dth)
	    dy = -dxy_average * sin(dth)
	    cos_th = cos(th)
	    sin_th = sin(th)
	    x += (dx * cos_th - dy * sin_th)
	    y += (dx * sin_th + dy * cos_th)
	th += dth 

	# Restore stuff into *self* (alphabetical order):
	self.previous_right_encoder_ = right_encoder
	self.previous_now_ = now
	self.previous_left_encoder_ = left_encoder
	self.th_ = th
	self.x_ = x
	self.y_ = y

	# Fill in the values for *quaternion* (quaternions are kind of magic):
	quaternion.x = 0.0 
	quaternion.y = 0.0
	quaternion.z = sin(th / 2.0)
	quaternion.w = cos(th / 2.0)
    
	# Send the odometry transform frame:
	odom_broadcaster.sendTransform(
	  (x, y, 0),
	  (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
	  now,
	  base_frame,
	  "odom")
    
	# Fill in *odom* and publish it:
	odom.header.frame_id = "odom"
	odom.child_frame_id = base_frame
	odom.header.stamp = now
	odom.pose.pose.position.x = x
	odom.pose.pose.position.y = y
	odom.pose.pose.position.z = 0.0
	odom.pose.pose.orientation = quaternion
	odom.pose.covariance = covariance
	odom.twist.twist.linear.x = vxy
	odom.twist.twist.linear.y = 0.0
	odom.twist.twist.angular.z = vth
	odom.twist.covariance = covariance
	odom_pub.publish(odom)

    def right_encoder_set(self, value, time):
	""" *Dead_Reckon*: Set the right encoder to *value* and *time*.
	"""

	# Verify argument types:
	assert isinstance(value, int)
	#assert isinstance(value, rospy.Time)

	# Load up *self*:
	self.right_encoder_ += value
	self.right_encoder_time_ = time

class Logger:
    def __init__(self, verbose=True):
	""" *Logger*: Initialize *self* with *verbose*.
	"""

	self.verbose_ = verbose

    def fatal(self, message):
	""" *Logger*: Log a fatal *message* and terminate program.
	"""

	self.warn(message)
	os._exit(1)

    def info(self, message):
	""" *Looger*: Log an information *message*.
	"""

	rospy.loginfo(message)
	if self.verbose_:
	    print(message)

    def warn(self, message):
	""" *Logger*: Log a warning *message*.
	"""

	rospy.loginfo(message)
	print(message)

class Sensor:
    """ *Sensor* is a base class for all sensors.
    """

    NO_TYPE = 0
    ENCODER_TYPE = 1
    SONAR_TYPE = 2

    def __init__(self, name, sensor_type, sensor_id, frame_id):
	""" *Sensor*: Initialize a *Sensor* object to contain *name*,
	    *sensor_type*, *id*, and *frame_id*.
	"""

	# Verify Argument Types:
	assert isinstance(name, str)
	assert isinstance(sensor_type, int)
	assert isinstance(sensor_id, int)
	assert isinstance(frame_id, str)

	# Load up *self*:
	self.name = name
	self.sensor_type = sensor_type
	self.sensor_id = sensor_id
	self.frame_id = frame_id

    def __format__(self, format):
	""" *Sensor*: Return a formated version of *self*.  For now, the
	    *format* argument is ignored.
	"""

	return "name:'{0}' type:{1} id:{2} frame_id:'{3}'".format(
	  self.name, self.sensor_type, self.sensor_id, self.frame_id)

class Encoder_Sensor(Sensor):
    """ *Encoder_Sensor*: An encoder sensor object represents motor encoder.
    """

    def __init__(self, dead_reckon, is_left):
	""" *Encoder_Sensor*: Initialize an *Encoder* object
	"""

	# Verify routine arguments:
	assert isinstance(dead_reckon, Dead_Reckon)
	assert isinstance(is_left, bool)

	name = "right_encoder"
	sensor_id = 1
	if is_left:
	    name = "left_encoder"
	    sensor_id = 0
	frame_id = "none"

	Sensor.__init__(self, name, Sensor.ENCODER_TYPE, sensor_id, frame_id)
	self.dead_reckon_ = dead_reckon
	self.is_left_ = is_left

    def value_set(self, value, time):
	""" *Encoder_Sensor*: Set the appropiate encoder to *value* and *time*:
	"""

	dead_reckon = self.dead_reckon_
	if self.is_left_:
	    dead_reckon.left_encoder_set(value, time)
	else:
	    dead_reckon.right_encoder_set(value, time)

class Sonar_Sensor(Sensor):
    def __init__(self, name, sensor_type, sensor_id, frame_id,
      field_of_view, min_range, max_range):
	""" *Sonar_Sensor*: Initialize a *Sonar_Sensor* to contain *name*,
	    *sensor_id*, *frame_id*, *field_of_view*, *min_range*, *max_range*.
	"""

	# Verify Argument Types:
	assert isinstance(name, str)
	assert isinstance(sensor_id, int)
	assert isinstance(frame_id, str)
	assert isinstance(field_of_view, float)
	assert isinstance(min_range, float)
	assert isinstance(max_range, float)

	# Load up *self*:
	Sensor.__init__(self, name, Sensor.SONAR_TYPE, sensor_id, frame_id)
	self.field_of_view_ = field_of_view
	self.min_range_ = min_range
	self.max_range_ = max_range
	self.msg_ = msg = Range()
	self.pub_ = rospy.Publisher("~sensor/{0}".format(name),
	  Range, queue_size=5)

	# Initialize *msg* a little more:
	msg.header.frame_id = frame_id
	msg.radiation_type = Range.ULTRASOUND
	msg.field_of_view = field_of_view
	msg.min_range = min_range
	msg.max_range = max_range

    def __format__(self, format):
	""" *Sonar_Sensor*: Return formated version of "self".  For now,
	    the *format* argument is ignored.
	"""

	# Verify arguments:
	assert isinstance(format, str)
	return "{0} field_of_view: {1} min_range: {2} max_range: {3}".format(
	  Sensor.__format__(self, ""),
	  self.field_of_view_, self.min_range_, self.max_range_)

    def value_set(self, value, time):
	""" *Sonar_Sensor*: Set the *value* and *time* for *self*
	    and publish it.
	"""

	# Verify routine arguments:
	assert isinstance(value, int)
	#assert isinstance(time, rospy.Time)

	# Convert from integer millimeters to float meters:
	range = float(value) / 1000.0

	# Load *value* and *time* into *msg*:
	msg = self.msg_
	msg.range = range
	msg.header.stamp = time

	# Publish *msg*:
	self.pub_.publish(msg)

# Fire off the program:
if __name__ == '__main__':
    main()
