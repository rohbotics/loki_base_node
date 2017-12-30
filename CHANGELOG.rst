^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bus_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-12-30)
------------------
* change package name from bus_server to loki_base_node
* Contributors: Rohan Agrawal

0.1.1 (2016-04-14)
------------------
* Added minor travis stuff
* Contributors: Rohan Agrawal

0.1.0 (2016-01-17)
------------------
* Added sonar statistics install rule
* Updated package.xml
* Added install rules
  to make sure python scripts are runnable in non-devel environments (such as installing from debs)
* Changed minimum sensor range to .15
* Fixed typo for /dev/ttyAMA0
* Merged
* Fixed /dev/ttyACM0=>/dev/ttyAMA0
* Use default ports if no port parameter is specified.
* Got dynamic sonar scheduling to work.
* Continuing to work on Sonars.
* Changed the strategy for managing clock skew between the robot and ROS.
* Added class/left_id/right_id fields to each Sonar.  These can be specified in the .yaml file and get reflected by bus_server.py to through to the robot firmware.  These fields are needed for dynamic sonar scheduling.
* Updated the descriptive text at the beginning of bus_server.py .
* Added Twist (cmd_vel) and transfer PID parameters to robot.
* Merge branch 'master' of https://github.com/UbiquityRobotics/bus_server
* Got odometry work in bus_server.py.
* Got odometry working.
* Refactored dead reckoning code into Dead_Reckon class.
* Merge branch 'master' of https://github.com/UbiquityRobotics/bus_server
* Backing down PID loop debug to minimal
* Merge branch 'master' of https://github.com/UbiquityRobotics/bus_server
* Got sonar sensers to publish range topics all the way to RViz.
* Increase PID rate and fix problem with initial going backwards at slow speeds. Also have debug serial to the bus_uart in PID loop which is on for now but can be disabled.
* Fixed typo with a duplicate left encoder set.
* Started adding sonar support to bus_server.py.
* Added in accidentally deleted code that sets encoders in PID code chunk.  Me bad.
* Rearranged bus_server.py to have all of the imports in one place.  Added main() and made the last line of code in the file run main().
* Created bus_server.py by concatenating all of the ROS Aduino bridge python files into one.  This code will not work.
* Added sonars to queue command.
* Added q command to bus server.  Some minor clean up on encoder interface.
* Making the Do PID param be Ko as other places like that term more.  No actual functional change with this cleanup
* Change PID loop rate to 10 hz and put command u for pin params in order of  Kp Kd Ki Do Ci to better match how it was before Ci appeared.  also if we use 'u 1' with less than 5 params we print params and don't set them.
* Removed concept of unit number for sonars.
* Refactored the code so that Sonar.{h,cpp} no longer needs RAB_Sonar{h,cpp}.  This touched a bunch of files.
* Removed pin definitions from bus_loki and Sonar.
* Removed unused and unneed encoder registers from bus_sonar10.
* Got bus_sonar10 to return ficticious sonar data over the bus.
* Refactored bus_server, bus_loki, and bus_freya to add the RAB_Sonar class.  This is prepatory work for bus_sonar10.
* Change pid update to 25hz from 5hz. Add cap for speeds in z command. Change the debug PID override mode so you must set debug flag  to enable PID override when Kp is 0 so we can better tune pid without it interfering.  IMPORTANT:  Adding integral cap term so the u command has 5 terms now as Kp Kd Ki Ci Do   where new term is Ci as integral cap to avoid runaway integral term issues
* Adding description of serial commands we support
* Adding description of serial commands we support
* Added a debug to support periodic print of sonar meas error counters for minimally evasive debug
* Return distances in Cm now per correct way ROS expects the values
* Switch the p command to read from cached in-ram prior measurements.  Added back in the o which does inline measurement.  Returned values for Sonar are still in mm.
* Changed 'o' command to 'p' command.
* Adding system_debug_flags for dynamic changing of what is printed to serial port.  Should act like before by default is the intention.  Flags in bus_server.h
* Fixes to non-PID motor control when Kp is set to 0 and support for reading all sonars or just one using o command
* Renamed x command to z command.  z command sets the motor speeds without using PID loop.
* Removed some debugging code.  Removed a minus sign on the right motor speed.
* Got Loki encoders working.
* Upgrade to new Bus_Motor_Encoder API that has encoder get/set and pwm set methods.
* Wrapped up Bridge object conversion.
* More conversion over to Bridge object.
* Started transition over to Bridge object.
* More Freya/Loki refactoring.
* Finished refactoring Freya code.
* Started refactoring Freya and Loki code.
* Remove serial input echo so ROS Arduino Bridge functions properly.  Added option to invert encoder sense but have left it to be functionally as it was before so it is a debug compile time option.
* Lot's of code refactoring.
* Got bus_server to talk to bus_bridge_encoders_sonar on the bus.
* Converted code into Bus_Server class.  Added in some serial port code from the Configurator project.
* Got bus_server_server to respond to messages from bus_bridge_encoders_sonar .
* Added missing Frame_Buffer.cpp/h
* Refactored sharable code from bus_raspberry_pi.
* Added BusInt{8,16,32}{Get,Set}.srv RCP messages.
* Initial commit.
* Contributors: Mark, Rohan Agrawal, Wayne C. Gramlich, Wayne Gramlich
