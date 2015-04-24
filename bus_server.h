// Copyright (c) 2014-2015 by Wayne C. Gramlich.  All rights reserved.
//
// This code is the shared code for the bridge software that lives
// in bridge boards like the bus_beaglebone and the bus_raspberry_pi.

#ifndef BUS_SERVER_H_INCLUDED
#define BUS_SERVER_H_INCLUDED 1

#include <Bus_Slave.h>
#include <Frame_Buffer.h>
#include <Bus_Motor_Encoder.h>

#define TEST_BUS_OUTPUT 1
#define TEST_BUS_ECHO 2
#define TEST_BUS_COMMAND 3
#define TEST_BUS_BRIDGE 4
#define TEST_BUS_LINE 5

// Pin defintions for bus_beaglebone:
#define BUS_STANDBY       3
#define LED		 13

#define ADDRESS 0x21
#define LED_GET 0
#define LED_PUT 1
#define BUFFER_SIZE 16

// The *Bus* object is defined here:
extern NULL_UART null_uart;
extern AVR_UART *bus_uart;
extern AVR_UART *debug_uart;
extern AVR_UART *host_uart;
extern Bus_Slave bus_slave;

// Set the *LED* to the value of *led*:
extern void motor_speeds_set(Byte left_speed, Byte right_speed);
extern void led_set(Logical led);
extern void led_blink(UShort on, UShort off);
extern void do_pid(Bus_Motor_Encoder *pid);
extern void pid_update();
extern void bridge_host_to_bus();
extern void bridge_setup(UByte test);
extern void bridge_loop(UByte test);

#endif // BUS_SERVER_H_INCLUDED
