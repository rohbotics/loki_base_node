// Copyright (c) 2014-2015 by Wayne C. Gramlich.  All rights reserved.
//

#include "Bus_Slave.h"
#include "Frame_Buffer.h"
#include "bus_server.h"

#define TEST_BUS_OUTPUT 1
#define TEST_BUS_ECHO 2
#define TEST_BUS_COMMAND 3
#define TEST_BUS_BRIDGE 4
#define TEST_BUS_LINE 5

// The *Bus_Slave* object is defined here:
NULL_UART null_uart;
AVR_UART *bus_uart = &avr_uart1;
AVR_UART *debug_uart = &avr_uart0;
AVR_UART *host_uart = &avr_uart0;
Bus_Slave bus_slave((UART *)bus_uart, (UART *)host_uart);

// The two PID set points are defined here:
SetPointInfo leftPID, rightPID;

// Set the *LED* to the value of *led*:
void led_set(Logical led) {
  if (led) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
}

void led_blink(UShort on, UShort off) {
  while (1) {
    led_set((Logical)1);
    delay(on);
    led_set((Logical)0);
    delay(off);
  }
}

static UByte address = 33;

static Short Kp = 20;	// PID Proportional Constant
static Short Kd = 12;	// PID Differential Constant
static Short Ki = 0;	// PID Integal Constant
static Short Ko = 50;	// PID common denOminator 
static Byte const MAX_PWM = 127;

static Logical is_moving = (Logical)0;

void motor_speeds_set(Byte left_speed, Byte right_speed) {
    //is_moving = (Logical)(left_speed != 0) || (right_speed != 0);
    bus_slave.command_byte_put(address,  9, left_speed);
    bus_slave.flush();
    bus_slave.command_byte_put(address, 11, right_speed);
    bus_slave.flush();
}

void pid_reset(SetPointInfo *pid){
   pid->TargetTicksPerFrame = 0.0;
   // Leave *encoder* field alone:
   //pid->Encoder = 0;
   pid->PrevEnc = leftPID.Encoder;
   pid->output = 0;
   pid->PrevInput = 0;
   pid->ITerm = 0;
}

void do_pid(SetPointInfo *pid) {
  Integer Perror;
  Integer output;
  Short input;

  //Perror = pid->TargetTicksPerFrame - (pid->Encoder - pid->PrevEnc);
  input = pid->Encoder - pid->PrevEnc;
  Perror = pid->TargetTicksPerFrame - input;

  // Avoid derivative kick and allow tuning changes, see:
  //
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  //   http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/

  //output =
  // (Kp * Perror + Kd * (Perror - pid->PrevErr) + Ki * pid->Ierror) / Ko;
  // p->PrevErr = Perror;
  output = (Kp * Perror - Kd * (input - pid->PrevInput) + pid->ITerm) / Ko;
  pid->PrevEnc = pid->Encoder;

  output += pid->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    // allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
    pid->ITerm += Ki * Perror;

  pid->output = output;
  pid->PrevInput = input;
}

void pid_update() {
  static Byte last_left_speed = 0x80;
  static Byte last_right_speed = 0x80;

  if (is_moving) {
    // Read the encoders:
    leftPID.Encoder = bus_slave.command_integer_get(address, 2);
    rightPID.Encoder = bus_slave.command_integer_get(address, 4);
  
    // Do the PID for each motor:
    //debug_uart->string_print((Text)"+");
    do_pid(&rightPID);
    do_pid(&leftPID);

    /* Set the motor speeds accordingly */
    //debug_uart->string_print((Text)" l=");
    //debug_uart->integer_print((UInteger)leftPID.output);
    //debug_uart->string_print((Text)" r=");
    //debug_uart->integer_print((UInteger)rightPID.output);
    //debug_uart->string_print((Text)"\r\n");

    Byte left_speed = (Byte)leftPID.output;
    Byte right_speed = (Byte)rightPID.output;
    

    if (left_speed != last_left_speed) {
        bus_slave.command_byte_put(address,  9, left_speed);
	bus_slave.flush();
	last_left_speed = left_speed;
    } 
    if (right_speed != last_right_speed) {
	bus_slave.command_byte_put(address, 11, right_speed);
	bus_slave.flush();
	last_right_speed = right_speed;
    }

    is_moving = (Logical)(left_speed != 0) || (right_speed != 0);

    //motor_speeds_set((Byte)leftPID.output, (Byte)rightPID.output);
  } else {
    //debug_uart->string_print((Text)"-");

    // If we're not moving there is nothing more to do:
    // Reset PIDs once, to prevent startup spikes, see
    //    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    // PrevInput is considered a good proxy to detect
    // whether reset has already happened

    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0) {
	pid_reset(&leftPID);
	pid_reset(&rightPID);
    }
  }
}

void bridge_host_to_bus() {
  Frame_Buffer			bus_frame_in;
  Frame_Buffer			bus_frame_out;
  Frame_Buffer			bus_frame_pending;
  UShort 			echo_suppress = 0xfefe;
  UShort			high_bits = 0xfefe;
  Frame_Buffer			host_frame_in;
  Frame_Buffer			host_frame_out;
  led_set((Logical)1);
  delay(200);
  led_set((Logical)0);

  // Just keep looping:
  while (1) {
    // Do we have an 8-bit byte to read from the host?:
    if (host_uart->can_receive() && !host_frame_in.is_full()) {
      // Receive 8-bit byte and save it into *host_frame_in*:
      UShort frame = host_uart->frame_get();
      host_frame_in.append(frame);
    }

    // Do we have an 8-bit byte to send to the host?:
    if (!host_frame_out.is_empty() && host_uart->can_transmit()) {
      // Send *frame* up to host:
      UShort frame = host_frame_out.lop();
      host_uart->frame_put(frame);
    }

    // Do we have a 9-bit byte to read from the bus?:
    if (bus_slave.can_receive() && !bus_frame_in.is_full()) {
      // Recevie a bus *frame* and save it into *bus_frame_in*:
      UShort frame = bus_slave.frame_get();
      bus_frame_in.append(frame);
    }

    // Is there a pending 9-bit frame to send to bus?:
    if (!bus_frame_out.is_empty() && bus_slave.can_transmit()) {
      // Send *frame* out to the bus:
      UShort frame = bus_frame_out.lop();
      bus_slave.frame_put(frame);

      // Remember that we want to *echo_suppress* *frame*:
      echo_suppress = frame;
    }

    // Forward all non echo suppressed traffic back up to host:
    if (!bus_frame_in.is_empty()) {
      // Grab a *frame* from *bus_frame_in*:
      UShort frame = bus_frame_in.lop();
      if (echo_suppress == (UShort)0xfefe) {
	// We are in non-*echo_suppress* mode.  We just forward
	// the byte up to the host:

	// We need to ensure that the 9th bit is not set:
	if ((frame & 0xff00) != 0) {
	  // Since we are the master, the 9th bit should never
	  // be set.  We indicate our confusion by setting the
	  // *led* high and masking off the 9th bit:
	  led_blink(200, 800);
	}

	// Send *frame* up to host:
	host_frame_out.append(frame & 0xff);
      } else {
	// We are in *echo_suppress* mode:

	// We need to check to ensure that the frame we sent
	// (=*echo_suppress*) is the same as the one we received
	// *frame*:
	if (echo_suppress != frame) {
	  // Since this should never happen, we indicate our
	  // confusion by setting *led* to high:
	  led_blink(500, 500);
	}

	// Now clear *echo_suppress*:
        echo_suppress = 0xfefe;
      }
    }

    if (!host_frame_in.is_empty() && !bus_frame_out.is_full()) {
      // Grab *frame* from *host_frame_in*:
      UShort frame = host_frame_in.lop();
	    
      // frames 0xc0 ... 0xc7 are reserved for "special" operations:
      if ((frame & 0xf8) == 0xc0) {
	switch (frame & 7) {
	  case 0:
	  case 1:
	  case 2:
	  case 3:
	    // 1100 00hh: Set *high_bits*:
	    // Set *high_bits* to (hh << 7):
	    high_bits = (frame & 3) << 7;
	    break;
	  case 4:
	    // Do bus discovery scan:
	    break;
	  case 5:
	    // Do bus break:
	    break;
	  case 6:
	    // Reserved:
	    led_set((Logical)0);
	    digitalWrite(LED, LOW);
	    break;
	  case 7:
	    // Reserved:
	    led_set((Logical)1);
	    break;
	}
      } else {
	if (high_bits == 0xfefe) {
	  // *high_bits* is not set, just forward *frame* to bus:
	  bus_frame_out.append(frame);
	} else {
	  // *high_bits* is set, forward modified *frame* to bus:
	  frame = high_bits | (frame & 0x7f);
	  bus_frame_out.append(frame);
	  high_bits = 0xfefe;
	}
      }
    }
  }
}

void bridge_setup(UByte test) {
  // Initialize *avr_uart0* as a debugging port:
  host_uart->begin(16000000L, 115200L, (Character *)"8N1");

  // For debugging, dump out UART0 configuration registers:
  //avr_uart0.string_print((Character *)" A:");
  //avr_uart0.uinteger_print((UInteger)UCSR0A);
  //avr_uart0.string_print((Character *)" B:");
  //avr_uart0.uinteger_print((UInteger)UCSR0B);
  //avr_uart0.string_print((Character *)" C:");
  //avr_uart0.uinteger_print((UInteger)UCSR0C);
  //avr_uart0.string_print((Character *)" H:");
  //avr_uart0.uinteger_print((UInteger)UBRR0H);
  //avr_uart0.string_print((Character *)" L:");
  //avr_uart0.uinteger_print((UInteger)UBRR0L);
  //avr_uart0.string_print((Character *)"\r\n");

  // Turn the *LED* on:
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  // Initalize *avr_uart1* to talk to the bus:
  bus_uart->begin(16000000L, 500000L, (Character *)"9N1");

  // Force the standby pin on the CAN transeciever to *LOW* to force it
  // into active mode:
  pinMode(BUS_STANDBY, OUTPUT);
  digitalWrite(BUS_STANDBY, LOW);

  // For debugging, dump out UART1 configuration registers:
  //avr_uart0.string_print((Character *)" A:");
  //avr_uart0.uinteger_print((UInteger)UCSR1A);
  //avr_uart0.string_print((Character *)" B:");
  //avr_uart0.uinteger_print((UInteger)UCSR1B);
  //avr_uart0.string_print((Character *)" C:");
  //avr_uart0.uinteger_print((UInteger)UCSR1C);
  //avr_uart0.string_print((Character *)" H:");
  //avr_uart0.uinteger_print((UInteger)UBRR1H);
  //avr_uart0.string_print((Character *)" L:");
  //avr_uart0.uinteger_print((UInteger)UBRR1L);
  //avr_uart0.string_print((Character *)"\r\n");

  // Enable/disable interrupts as needed:
  switch (test) {
    case TEST_BUS_OUTPUT:
      host_uart->string_print((Character *)"\r\nbb_output:\r\n");
      host_uart->interrupt_set((Logical)0);
      bus_uart->interrupt_set((Logical)0);
      break;
    case TEST_BUS_ECHO:
      host_uart->string_print((Character *)"\r\nbb_echo:\r\n");
      host_uart->interrupt_set((Logical)0);
      bus_uart->interrupt_set((Logical)0);
      break;
    case TEST_BUS_COMMAND:
      host_uart->string_print((Character *)"\r\nbb_command:\r\n");
      host_uart->interrupt_set((Logical)1);
      bus_uart->interrupt_set((Logical)1);
      break;
    case TEST_BUS_BRIDGE:
      // No announce because we are talking to *host_uart*:
      //host_uart->string_print((Character *)"\r\nbb_bridge:\r\n");
      host_uart->interrupt_set((Logical)1);
      bus_uart->interrupt_set((Logical)1);
      break;
    case TEST_BUS_LINE:
      // No announce because we are talking to *host_uart*:
      host_uart->string_print((Character *)"\r\nbb_line:\r\n");
      host_uart->interrupt_set((Logical)1);
      bus_uart->interrupt_set((Logical)1);
      break;
  }
}

void bridge_loop(UByte test) {
  switch (test) {
    case TEST_BUS_LINE: {
      // Some constants:
      static const UInteger PID_RATE = 5;			// Hz.
      static const UInteger PID_INTERVAL = 1000 / PID_RATE;	// mSec.
      //static const UInteger AUTO_STOP_INTERVAL = 2000;	// mSec.

      // Some variables that need to be unchanged through each loop iteration:
      static UByte address = 33;
      static Integer arguments[4];
      static UByte arguments_index = 0;
      static Character command = ' ';
      static Logical have_number = (Logical)0;
      static Logical is_negative = (Logical)0;
      //static UInteger last_motor_command_time = 0;
      static UInteger next_pid = 0;
      static Integer number = 0;

      // We need to know what time it is *now*:
      UInteger now = millis();

      // Process commands one *character* at a time:
      if (host_uart->can_receive()) {
	// Grab the next character since we have it:
	Character character = (Character)host_uart->frame_get();

	// Echo the input:
	host_uart->frame_put((UShort)character);
	if (character == '\r') {
	  host_uart->frame_put((UShort)'\n');
	}

	// Parse *number* one *character* at a time:
        if (character == '-') {
	  // Remember if we have a negative sign:
	  is_negative = (Logical)1;
        } else if ('0' <= character && character <= '9') {
	  // Deal with next decimal digit:
	  number = number * 10 + (Integer)(character - '0');
	  have_number = (Logical)1;
	  //debug_uart->string_print((Text)"num=");
	  //debug_uart->integer_print((Integer)number);
	  //debug_uart->string_print((Text)" ");
	} else if (character == ' ' || character == ':' || character == '\r') {
	  // Space, colon, and carriage-return terminate a number:
	  if (have_number) {
	    // Deal with negative numbers:
	    if (is_negative) {
	      number = -number;
	      is_negative = (Logical)0;
	    }

	    // Stuff *number* into the *arguments* list:
	    arguments[arguments_index++] = number;
	    number = 0;
	    have_number = (Logical)0;

	    // Reset the *number* parser:
	    number = 0;
	    have_number = (Logical)0;
	    is_negative = (Logical)0;
	  }
        } else if ('a' <= character && character <= 'z') {
	  command = character;
	}

	// Execute a command if we get '\r':
	if (character == '\r') {
	  // Dispatch on *command* character:
	  switch (command) {
	    case 'e': {
	      // Read encoders ("e"):
	      Integer encoder0 = bus_slave.command_integer_get(address, 2);
	      Integer encoder1 = bus_slave.command_integer_get(address, 4);

	      host_uart->integer_print(encoder0);
	      host_uart->string_print((Text)" ");
	      host_uart->integer_print(encoder1);
	      host_uart->string_print((Text)"\r\n");
	      break;
	    }
	    case 'm': {
	      // Set motor speeds ("m left, right"):
	      Byte left_speed = (Byte)arguments[0];
	      Byte right_speed = (Byte)arguments[1];
	      
	      //bus.command_byte_put(address, 9, left_speed);
	      //Byte xleft_speed = bus.command_byte_get(address, 8);
	      //bus.command_byte_put(address, 11, right_speed);
	      //Byte xright_speed = bus.command_byte_get(address, 10);

	      //motor_speeds_set(left_speed, right_speed);
	      //last_motor_command_time = now;

	      // For debugging:
	      //debug_uart->integer_print((Integer)left_speed);
	      //debug_uart->string_print((Text)"==");
	      //debug_uart->integer_print((Integer)xleft_speed);
	      //debug_uart->string_print((Text)" ");

	      //debug_uart->integer_print((Integer)right_speed);
	      //debug_uart->string_print((Text)"==");
	      //debug_uart->integer_print((Integer)xright_speed);
	      //debug_uart->string_print((Text)" ");

	      // For PID code:
	      is_moving = (Logical)(left_speed != 0 || right_speed != 0);
	      if (is_moving) {
		leftPID.TargetTicksPerFrame = left_speed;
		rightPID.TargetTicksPerFrame = right_speed;
	      } else {
	        motor_speeds_set(0, 0);
	      }

	      // Print the usual "OK" result:
	      host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'r': {
	      // Reset encoders ("r"):
	      bus_slave.command_integer_put(address, 3, 0);
	      bus_slave.command_integer_put(address, 5, 0);

	      // Print the usual "OK" result:
	      host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'u': {
	      // Update PID constants ("U Kp Kd Ki Ko");
	      Kp = arguments[0];
	      Kd = arguments[1];
	      Ki = arguments[2];
	      Ko = arguments[3];
	      host_uart->string_print((Text)"OK\r\n");

	      // For debugging:
	      debug_uart->integer_print(Kp);
	      host_uart->string_print((Text)" ");
	      debug_uart->integer_print(Kd);
	      host_uart->string_print((Text)" ");
	      debug_uart->integer_print(Ki);
	      host_uart->string_print((Text)" ");
	      debug_uart->integer_print(Ko);

	      // Print the usual "OK" result:
	      host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    default: {
	      host_uart->string_print((Text)"Invalid Command\r\n");
	      break;
	    }
	  }
	  command = '?';

	  // Reset the *arguments_index* for the next command:
	  arguments_index = 0;
	}
      }

      // Do we need to do a PID update cycle?:
      if (now > next_pid) {
	//debug_uart->string_print((Text)"+");
	pid_update();
        next_pid += PID_INTERVAL;
      }

      // Do we need to shut down the motor?:
      //if (now - last_motor_command_time > AUTO_STOP_INTERVAL) {
	//motor_speeds_set(0, 0);
      //}
      break;
    }
    case TEST_BUS_BRIDGE: {
      bridge_host_to_bus();
      break;
    }
    case TEST_BUS_COMMAND: {
      // Blink the *LED* some:

      // Set the *LED* to *HIGH* and then wait a little:
      bus_slave.command_ubyte_put(ADDRESS, LED_PUT, HIGH);
      Logical led_get = bus_slave.command_ubyte_get(ADDRESS, LED_GET);
      led_set(led_get);
      delay(100);

      // Set the *LED* to *LOW* and then wait a little:
      bus_slave.command_ubyte_put(ADDRESS, LED_PUT, LOW);
      led_get = bus_slave.command_ubyte_get(ADDRESS, LED_GET);
      led_set(led_get);
      delay(100);

      break;
    }
    case TEST_BUS_ECHO: {
      // *character* is static so that it does not change after each
      // iteration through *loop*:
      static Character character = '@';

      // Make sure that *character* is between '@' and '_':
      if (character < '@' || character > '_') {
	character = '@';
      }

      // Output *character* to bus:
      bus_slave.frame_put((UShort)character);

      // Get the resulting *echo_frame* and indicate when it does not match:
      UShort echo_frame = bus_slave.frame_get();
      if ((UShort)character != echo_frame) {
	host_uart->string_print((Character *)"!");
      }

      // Wait for the result from the remote module:
      UShort remote_frame = bus_slave.frame_get();

      // Print the *remote_frame* out to *host_uart*:
      host_uart->frame_put(remote_frame);

      // Print out any needed CRLF and update to next *character*:
      if (remote_frame == (UShort)'_') {
	host_uart->string_print((Character *)"\r\n");
	character = '@';
      } else {
	character += 1;
      }

      // Let's blink the LED for a little:
      led_set((remote_frame & 1) == 0);
      delay(100);

      led_set((remote_frame & 1) != 0);
      delay(100);

      break;
    }
    case TEST_BUS_OUTPUT: {
      // This verision of loop simply outputs 8-bit characters (in 9-bit mode)
      // to the bus starting from '@' to '_' and repeating.  The primary purpose
      // is to verify that both UART's are properly initialized to reasonable
      // baud rates.  We also ensure that the bus is terminated and the
      // CAN bus transceiver is on.

      UShort delay_milliseconds = 10;

      // *character* is a static variable:
      static Character character = 'U';

      // Make sure *character* is "in bounds":
      if (character < '@' || character > '_') {
	character = '@';
      }

      // Output *character* to bus:
      bus_slave.frame_put((UShort)character);

      // Set LED to be the same as the LSB of *frame*:
      led_set((character & 1) != 0);
      delay(delay_milliseconds);

      // Output *frame* back to user for debugging:
      debug_uart->frame_put((UShort)character);
      if (character >= '_') {
	// For debugging, dump out UART1 configuration registers:
	//debug_uart->string_print((Character *)" A:");
	//debug_uart->uinteger_print((UInteger)UCSR1A);
	//debug_uart->string_print((Character *)" B:");
	//debug_uart->uinteger_print((UInteger)UCSR1B);
	//debug_uart->string_print((Character *)" C:");
	//debug_uart->uinteger_print((UInteger)UCSR1C);
	//debug_uart->string_print((Character *)" H:");
	//debug_uart->uinteger_print((UInteger)UBRR1H);
	//debug_uart->string_print((Character *)" L:");
	//debug_uart->uinteger_print((UInteger)UBRR1L);
	//debug_uart->string_print((Character *)"\r\n");

	debug_uart->string_print((Character *)"\r\n");
        character = '@';
      } else {
	//character += 1;
      }

      // Set LED to be the opposite of the *frame* LSB:
      led_set((character & 1) == 0);
      delay(delay_milliseconds);

      break;
    }
  }
}

