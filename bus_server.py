#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

# Put all of the imports into alphabetical order:
from geometry_msgs.msg import Quaternion, Pose, Twist
from math import cos, degrees, pi as PI, radians, sin
from nav_msgs.msg import Odometry
import rospy
import os
from sensor_msgs.msg import Range
from serial.serialutil import SerialException
from serial import Serial
import sys
from tf.broadcaster import TransformBroadcaster
import thread
import time
import traceback

#from ros_arduino_python.arduino_driver import Arduino
#from ros_arduino_python.arduino_sensors import *
#from ros_arduino_msgs.srv import *
#from ros_arduino_python.base_controller import BaseController

#import roslib; roslib.load_manifest('ros_arduino_python')
#from ros_arduino_msgs.msg import *
#import roslib; roslib.load_manifest('ros_arduino_python')


LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1

SERVO_MAX = 180
SERVO_MIN = 0

def main():
    print("Hello")

class ArduinoROS():
    def __init__(self):
        rospy.init_node('Arduino', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        self.use_base_controller = rospy.get_param("~use_base_controller", False)
        
        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # The SensorState publisher periodically publishes the values of all sensors on
        # a single topic.
        self.sensorStatePub = rospy.Publisher('~sensor_state', SensorState, queue_size=5)
        
        # A service to position a PWM servo
        rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)
        
        # A service to read the position of a PWM servo
        rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)
        
        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)
        
        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)
       
	# A service to set pwm values for the pins
	rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)

	# Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Initialize any sensors
        self.mySensors = list()
        
        sensor_params = rospy.get_param("~sensors", dict({}))
        
        for name, params in sensor_params.iteritems():
            # Set the direction to input if not specified
            params = params.copy()
            try:
                params['direction']
            except:
                params['direction'] = 'input'
            frame_id = params.pop('frame_id', self.base_frame)
            
                
            if params['type'] == "Ping":
                sensor = Ping(self.controller, name, params['pin'], params['rate'], frame_id,
                  direction=params.pop('direction'), field_of_view=params.pop('field_of_view', 0.785398163),
                  min_range=params.pop('min_range', 0.02), max_range=params.pop('max_range', 3.0))
            elif params['type'] == "GP2D12":
                sensor = GP2D12(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'PololuMotorCurrent':
                sensor = PololuMotorCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsVoltage':
                sensor = PhidgetsVoltage(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsCurrent':
                sensor = PhidgetsCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
                
#                if params['type'] == "MaxEZ1":
#                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
#                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']

            self.mySensors.append(sensor)
            rospy.loginfo(name + " " + str(params))
              
        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame)
    
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            for sensor in self.mySensors:
                mutex.acquire()
                sensor.poll()
                mutex.release()
                    
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()
            
            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()
            
            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp = now
                for i in range(len(self.mySensors)):
                    msg.name.append(self.mySensors[i].name)
                    msg.value.append(self.mySensors[i].value)
                try:
                    self.sensorStatePub.publish(msg)
                except:
                    pass
                
                self.t_next_sensors = now + self.t_delta_sensors
            
            r.sleep()
    
    # Service callback functions
    def ServoWriteHandler(self, req):
        self.controller.servo_write(req.id, req.value)
        return ServoWriteResponse()
    
    def ServoReadHandler(self, req):
        self.controller.servo_read(req.id)
        return ServoReadResponse()
    
    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()
    
    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()
              
    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()
 
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Arduino Node...")
        
"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

"""

class Arduino:
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
	self.motors_reversed = False
	self.left_motor_reversed = False
	self.right_motor_reversed = False
    
        # Keep things thread safe
        self.mutex = thread.allocate_lock()
            
        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()   
                if test != self.baudrate:
                    raise SerialException
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self): 
        ''' Close the serial port.
        '''
        self.port.close() 
    
    def motors_configure(self, motors_reversed = False,
      left_motor_reversed = False, right_motor_reversed = False):
	self.motors_reversed = motors_reversed
	self.left_motor_reversed = left_motor_reversed
	self.right_motor_reversed = right_motor_reversed

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands   
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.read(1)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        #rospy.loginfo("Send command '{0}' to arduino".format(cmd))

        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            value = self.recv(self.timeout)
            while attempts < ntries and (value == '' or value == 'Invalid Command' or value == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    value = self.recv(self.timeout)
                except:
		    rospy.loginfo("Command '{0}' triggered exception".
		      format(cmd))
                    print "Exception executing command: " + cmd
                attempts += 1
        except:
            self.mutex.release()
	    rospy.loginfo("Command '{0}' triggered exception".format(cmd))
            print "Exception executing command: " + cmd
            value = None
        
        self.mutex.release()

	try:
	    value = int(value)
	    rospy.loginfo("Command '{0}' returned {1}".format(cmd, value))
	except:
	    rospy.loginfo("Command '{0}' got '{1}' instead of integer".
	      format(cmd, value))
            value = 0
        return value

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []
        
        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        rospy.loginfo("Send command '{0}' to arduino".format(cmd))
        self.mutex.acquire()
        
        try:
            self.port.flushInput()
        except:
            pass
        
        ntries = 1
        attempts = 0
        
        try:
            self.port.write(cmd + '\r')
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.port.flushInput()
                    self.port.write(cmd + '\r')
                    ack = self.recv(self.timeout)
                except:
                    print "Exception executing command: " + cmd
            attempts += 1
        except:
            self.mutex.release()
            print "execute_ack exception when executing", cmd
            print sys.exc_info()
            return 0
        
        self.mutex.release()
        return ack == 'OK'   
    
    def update_pid(self, Kp, Kd, Ki, Ko, Ci):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko) + ':' + str(Ci)
        self.execute_ack(cmd)                          

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return int(self.execute('b'));

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
	    if self.motors_reversed:
		values[0], values[1] = values[1], values[0]
	    if self.left_motor_reversed:
		values[0] = -values[0]
	    if self.right_motor_reversed:
		values[1] = -values[1]
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')
    
    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
	if self.left_motor_reversed:
	    left = -left
	if self.right_motor_reversed:
	    right = -right
	if self.motors_reversed:
	    left, right = right, left

        return self.execute_ack('m %d %d' %(right, left))
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)
            
    def analog_read(self, pin):
        return self.execute('a %d' %pin)
    
    def analog_write(self, pin, value):
        return self.execute_ack('x %d %d' %(pin, value))
    
    def digital_read(self, pin):
        return self.execute('d %d' %pin)
    
    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))
    
    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''        
        return self.execute_ack('s %d %d' %(id, min(SERVO_MAX, max(SERVO_MIN, degrees(pos)))))
    
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''        
        return radians(self.execute('t %d' %id))

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return self.execute('p %d' %pin);
    
#    def get_maxez1(self, triggerPin, outputPin):
#        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
#            sensor connected to the General Purpose I/O lines, triggerPin, and
#            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
#            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
#            power up, otherwise it wont range correctly for object less than 6
#            inches away! The sensor reading defaults to use English units
#            (inches). The sensor distance resolution is integer based. Also, the
#            maxsonar trigger pin is RX, and the echo pin is PW.
#        '''
#        return self.execute('z %d %d' %(triggerPin, outputPin)) 
 

""" Basic test for connectivity """
if False:
    if os.name == "posix":
        portName = "/dev/ttyACM0"
    else:
        portName = "COM43" # Windows style COM port.
        
    baudRate = 57600

    myArduino = Arduino(port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()
     
    print "Sleeping for 1 second..."
    time.sleep(1)   
    
    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()
    
    print "Connection test successful.",
    
    myArduino.stop()
    myArduino.close()
    
    print "Shutting down Arduino."

"""
    Sensor class for the arudino_python package
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

    
class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5
    
class Sensor(object):
    def __init__(self, controller, name, pin, rate, frame_id, direction="input", **kwargs):
        self.controller = controller
        self.name = name
        self.pin = pin
        self.rate = rate
        self.direction = direction

        self.frame_id = frame_id
        self.value = None
        
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if self.direction == "input":
                try:
                    self.value = self.read_value()
                except:
                    return
            else:
                try:
                    self.ack = self.write_value()
                except:
                    return          
    
            # For range sensors, assign the value to the range message field
            if self.message_type == MessageType.RANGE:
                self.msg.range = self.value
            else:
                self.msg.value = self.value

            # Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            
            self.t_next = now + self.t_delta
    
class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Analog, queue_size=5)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.analog_read(self.pin)
    
    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)
    
class AnalogFloatSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, AnalogFloat, queue_size=5)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.analog_read(self.pin)
    
    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)
    
        
class DigitalSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(DigitalSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.BOOL
        
        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Digital, queue_size=5)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.digital_read(self.pin)
    
    def write_value(self):
        # Alternate HIGH/LOW when writing at a fixed rate
        self.value = not self.value
        return self.controller.digital_write(self.pin, self.value)
    
    
class RangeSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(RangeSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.RANGE
        
        self.msg = Range()
        self.msg.header.frame_id = kwargs.pop('frame_id', self.frame_id)
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Range, queue_size=5)
        
    def read_value(self):
        self.msg.header.stamp = rospy.Time.now()

        
class SonarSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(SonarSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.ULTRASOUND
        
        
class IRSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(IRSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.INFRARED
        
class Ping(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(Ping, self).__init__(*args, **kwargs)
                
        self.msg.field_of_view = kwargs.pop('field_of_view', 0.785398163)
        self.msg.min_range = kwargs.pop('min_range', 0.02)
        self.msg.max_range = kwargs.pop('max_range', 3.0)
        
    def read_value(self):
        # The Arduino Ping code returns the distance in centimeters
        mm = self.controller.ping(self.pin)
        
        # Convert it to meters for ROS
        distance = mm / 1000.0

	if distance > 3.0:
	    distance = 3.0
        return distance
    
        
class GP2D12(IRSensor):
    def __init__(self, *args, **kwargs):
        super(GP2D12, self).__init__(*args, **kwargs)
        
        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80
        
    def read_value(self):
        value = self.controller.analog_read(self.pin)
        
        if value <= 3.0:
            return self.msg.max_range
        
        try:
            distance = (6787.0 / (float(value) - 3.0)) - 4.0
        except:
            return self.msg.max_range
            
        # Convert to meters
        distance /= 100.0
        
        # If we get a spurious reading, set it to the max_range
        if distance > self.msg.max_range: distance = self.msg.max_range
        if distance < self.msg.min_range: distance = self.msg.max_range
        
        return distance
    
class PololuMotorCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PololuMotorCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Pololu source code
        milliamps = self.controller.analog_read(self.pin) * 34
        return milliamps / 1000.0
    
class PhidgetsVoltage(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsVoltage, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation
        voltage = 0.06 * (self.controller.analog_read(self.pin) - 500.)
        return voltage
    
class PhidgetsCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation for the 20 amp DC sensor
        current = 0.05 * (self.controller.analog_read(self.pin) - 500.)
        return current
    
class MaxEZ1Sensor(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(MaxEZ1Sensor, self).__init__(*args, **kwargs)
        
        self.trigger_pin = kwargs['trigger_pin']
        self.output_pin = kwargs['output_pin']
        
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        return self.controller.get_MaxEZ1(self.trigger_pin, self.output_pin)

            
if False:
    myController = Controller()
    mySensor = SonarSensor(myController, "My Sonar", type=Type.PING, pin=0, rate=10)
            
"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame):
        self.arduino = arduino
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") 
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "") 
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        pid_params['Kp'] = rospy.get_param("~Kp", 20)
        pid_params['Kd'] = rospy.get_param("~Kd", 3)
        pid_params['Ki'] = rospy.get_param("~Ki", 86)
        pid_params['Ko'] = rospy.get_param("~Ko", 300)
        pid_params['Ci'] = rospy.get_param("~Ci", 100)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
	self.left_motor_reversed = rospy.get_param("~left_motor_reversed", False)
	self.right_motor_reversed = rospy.get_param("~right_motor_reversed", False)
	arduino.motors_configure(self.motors_reversed, self.left_motor_reversed, self.right_motor_reversed)

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        self.Ci = pid_params['Ci']
        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko, self.Ci)

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            # Read the encoders
            try:
                left_enc, right_enc = self.arduino.get_encoder_counts()
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                dright = 0
                dleft = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

            self.enc_right = right_enc
            self.enc_left = left_enc
            
            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt
                
            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 
    
            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
    
            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.pose.covariance = [0.2,  0,    0,     0,     0,     0,
                                    0,    0.2,  0,     0,     0,     0,
                                    0,    0,    0.2,   0,     0,     0,
                                    0,    0,    0,     0.2,   0,     0,
                                    0,    0,    0,     0,     0.2,   0,
                                    0,    0,    0,     0,     0,     0.2]
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth
            odom.twist.covariance = [0.2,  0,    0,     0,     0,     0,
                                     0,    0.2,  0,     0,     0,     0,
                                     0,    0,    0.2,   0,     0,     0,
                                     0,    0,    0,     0.2,   0,     0,
                                     0,    0,    0,     0,     0.2,   0,
                                     0,    0,    0,     0,     0,     0.2]
            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_left = 0
                self.v_des_right = 0
                
            if self.v_left < self.v_des_left:
                self.v_left += self.max_accel
                if self.v_left > self.v_des_left:
                    self.v_left = self.v_des_left
            else:
                self.v_left -= self.max_accel
                if self.v_left < self.v_des_left:
                    self.v_left = self.v_des_left
            
            if self.v_right < self.v_des_right:
                self.v_right += self.max_accel
                if self.v_right > self.v_des_right:
                    self.v_right = self.v_des_right
            else:
                self.v_right -= self.max_accel
                if self.v_right < self.v_des_right:
                    self.v_right = self.v_des_right
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_left, self.v_right)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        rospy.loginfo("CmdVel: x={0}m/s th={1}deg/s".format(x, th * 180.0/pi))

        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)
        
if __name__ == '__main__':
    main()
