#!/usr/bin/env python

# Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

"""

Print statics about sonars.

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
    sonar_statistics = Sonar_Statistics()
    sonar_statistics.poll_loop()

class Sonar_Statistics:
    """ *Sonar_Statistics*: Initialize.
    """

    def __init__(self):
	""" *Bus_Server*: Initialize the bus server.
	"""

	# Initialize the *logger*:
	self.logger_ = logger = Logger(True)
        self.mutex_ = mutex = thread.allocate_lock()

	# Create the ROS initializion node:
	rospy.init_node("sonar_statistics", log_level=rospy.DEBUG)
	logger.info("Starting sonar_statistics...")

	# 
	self.poll_rate_ = poll_rate = rospy.get_param("~poll_rate", 5)
	self.rate_ = rospy.Rate(1.0 / float(poll_rate))
	self.counts_ = counts = []

	# Clean up when terminating the node:
	rospy.on_shutdown(self.shut_down)

	# ...
	for index in range(16):
	    rospy.Subscriber("/arduino/sensor/sonar_{0}".format(index),
	      Range, self.range_callback)
	    counts.append(0)

    def range_callback(self, request):
	""" *Sonar_Statistics*: This is routine is called each time a *Range*
	    message is received.
	"""

	mutex = self.mutex_
        mutex.acquire()

	index = int(request.header.frame_id.split('_')[1])

	logger = self.logger_
	print("{0} {1:02d}".format(index * "   ", index))

        self.counts_[index] += 1

	mutex.release()

    def poll_loop(self):
	""" *Bus_Server*: Perform polling for the *Bus_Server* object.
	"""

	logger = self.logger_
	poll_rate = self.poll_rate_
	mutex = self.mutex_

	# Define some types and routines for Time management and manipulation:
	Duration = rospy.Duration
	Time = rospy.Time
	now_routine = rospy.Time.now
	next_poll_time = now_routine()
	rate = self.rate_
	counts = self.counts_

	# Compute *delta_poll_time* as the reciprical of *poll_rate*:
	delta_poll_time = rospy.Duration(1.0 / float(poll_rate))

	# Start polling the sensors and base controller
	while not rospy.is_shutdown():
	    # Figure out if it is time to poll the sensor queue:
	    now = now_routine()
	    if now >= next_poll_time:
		# Update *next_poll_time*:
		next_poll_time += delta_poll_time

	    # Print out a histogram of *counts*:

	    # Step 1: compute *counts_maximum* and *counts_total*:
	    mutex.acquire()
	    counts_maximum = 0
	    counts_total = 0
	    for count in counts:
		counts_maximum = max(counts_maximum, count)
		counts_total += count

	    # Print the sonar numbers:
	    line = ""
	    for index in range(len(counts)):
		line += " {0:02d}".format(index)
	    print(line)

	    # Print the first summary line:
	    line = ""
	    for count in counts:
		line += " {0:2d}".format(count)
	    line += " Total {0}".format(counts_total)
	    print(line)

	    # Print the actual histogram one line at a time:
            for line_index in range(counts_maximum):
		# Assemble one *line* of historgram:
		line = ""
		for index in range(len(counts)):
		    if line_index >= counts[index]:
			line += "   "
		    else:
			line += " **"

		# Print out one histogram *line*:
		print(line)

	    # Step 3: Zero out *counts*:
	    for index in range(len(counts)):
		counts[index] = 0
            mutex.release()
            
	    # Sleep until next time:
	    rate.sleep()
    
    def shut_down(self):
	""" *Sonar_Statistics*: Shut down the bus_server.
	"""

	logger = self.logger_
        logger.warn("Shutting down")

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

# Fire off the program:
if __name__ == '__main__':
    main()
