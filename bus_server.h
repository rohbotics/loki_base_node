// Copyright (c) 2014-2015 by Wayne C. Gramlich.  All rights reserved.
//
// This code is the shared code for the bridge software that lives
// in bridge boards like the bus_beaglebone and the bus_raspberry_pi.

#ifndef BUS_SERVER_H_INCLUDED
#define BUS_SERVER_H_INCLUDED 1

#include <Bus_Slave.h>
#include <Frame_Buffer.h>
#include <Bus_Motor_Encoder.h>
#include <RAB_Sonar.h>

#define TEST_BUS_OUTPUT 1
#define TEST_BUS_ECHO 2
#define TEST_BUS_COMMAND 3
#define TEST_BUS_BRIDGE 4
#define TEST_RAB_FREYA 5	// RAB == ROS Arduino Bridge
#define TEST_RAB_LOKI 6

// Setup control for debug printouts that we can manage live
// Binary bits in a flag word default is to act like code of May 2015
#define DBG_FLAG_PID_DISABLE_OK     0x0001   // Allow disable of PID control when Kp = 0
#define DBG_FLAG_USENSOR_ERR_DEBUG  0x0002   // Show error counters every so often
#define DBG_FLAG_USENSOR_DEBUG      0x0004   // Show ultrasonic sensor acquision debug info
#define DBG_FLAG_USENSOR_RESULTS    0x0008   // Show all ultrasonic sensor values at end of each pass
#define DBG_FLAG_ECHO_INPUT_CHARS   0x0010   // Echo input for manual ease of use
#define DBG_FLAG_MOTOR_SETTINGS     0x0020   // Show values going to motor controllers
#define DBG_FLAG_PID_DEBUG          0x0040   // Show values for Pid debug loop
#define DBG_FLAG_PARAMETER_SETUP    0x0100   // Showing info as we set general parameters 
#define DBG_FLAG_UART_SETUP         0x0200   // Showing info for registers of uart

class Protocol {
  virtual void baud_rate() = 0;
  virtual void motors_speed_set(Short left_speed, Short right_speed);
  virtual void pid_parameter_set(Short proportional,
    Short integral, Short derivative, Short denominator) = 0;
  virtual Integer left_encoder_get() = 0;
  virtual Integer right_encoder_get() = 0;
  virtual Integer encoders_reset();
};

class Bridge {
  public:
    Bridge(AVR_UART *host_uart, AVR_UART *bus_uart, AVR_UART *debug_uart,
     Bus_Slave *bus_slave, Bus_Motor_Encoder *left_motor_encoder,
     Bus_Motor_Encoder *right_motor_encoder, RAB_Sonar *rab_sonar);
    void pid_update(UByte mode);
    void host_to_bus();
    void setup(UByte mode);
    void loop(UByte mode);
    void motor_speeds_set(Short left_speed, Short right_speed);
    void pid_debug_print_1(Logical verbose);
    void pid_debug_print_2();
  private:
    Bus_Slave *_bus_slave;
    AVR_UART *_bus_uart;
    AVR_UART *_debug_uart;
    AVR_UART *_host_uart;
    Logical _is_moving;
    Bus_Motor_Encoder *_left_motor_encoder;
    Bus_Motor_Encoder *_right_motor_encoder;
    RAB_Sonar *rab_sonar_;
    UInteger time_base_;
    UInteger previous_left_encoder_;
    UInteger previous_right_encoder_;
};
#endif // BUS_SERVER_H_INCLUDED

