// Copyright (c) 2014-2015 by Wayne C. Gramlich.  All rights reserved.

#include "Bus_Slave.h"
#include "Frame_Buffer.h"
#include "bus_server.h"


// If we want to reverse raw encoder polarity it can be done here
#define ENCODER_RIGHT_POLARITY   (1)
#define ENCODER_LEFT_POLARITY    (1)

#define PID_OVERRIDE_FACTOR 	 (15)   // Factor used if in PID override mode
#define PID_OVERRIDE_OFFSET      (30)   // An offset to add AFTER scaling to PID values


// Trigger and readback delay from an HC-SR04 Ulrasonic Sonar, return in meters
extern float usonar_inlineReadMeters(int sonarUnit);
extern int   usonar_getLastDistInMm(int sonarUnit);

// *Bridge* methods:

Bridge::Bridge(AVR_UART *host_uart, AVR_UART *bus_uart, AVR_UART *debug_uart,
  Bus_Slave *bus_slave, Bus_Motor_Encoder *left_motor_encoder,
  Bus_Motor_Encoder *right_motor_encoder) {
  _left_motor_encoder = left_motor_encoder;
  _right_motor_encoder = right_motor_encoder;
  _bus_slave = bus_slave;
  _host_uart = host_uart;
  _bus_uart = bus_uart;
  _debug_uart = debug_uart;
  _is_moving = (Logical)0;
}

void Bridge::pid_update(UByte mode) {
  //_debug_uart->string_print((Text)"[");
  static Byte last_left_speed = 0x80;
  static Byte last_right_speed = 0x80;
  //static Integer previous_left_encoder = 0;

  if (_is_moving) {
    // Read the encoders:
    //_debug_uart->string_print((Text)"a");
    Integer left_encoder =
     _left_motor_encoder->encoder_get() * ENCODER_LEFT_POLARITY;
    Integer right_encoder =
     _right_motor_encoder->encoder_get() * ENCODER_RIGHT_POLARITY;

    // Load the encoder values into the encoder porition (this API is silly):
    _left_motor_encoder->encoder__set(left_encoder);
    _right_motor_encoder->encoder__set(right_encoder);
  
    // Do the PID for each motor:
    //debug_uart->string_print((Text)"b");

    if ((system_debug_flags_get() & DBG_FLAG_PID_DISABLE_OK) &&
        // Do normal 'm' command unless Kp are zero, then do direct pwm set
        (_left_motor_encoder->proportional_get() == 0) && 
        (_right_motor_encoder->proportional_get() == 0))  {
        // bypass PID code if Kp are all zero
    } else {
        if (system_debug_flags_get() & DBG_FLAG_PID_DEBUG) {
          _host_uart->string_print((Text)"P\r\n");
        }
        _right_motor_encoder->do_pid();
        _left_motor_encoder->do_pid();
    } 

    /* Set the motor speeds accordingly */
    //_debug_uart->string_print((Text)" l=");
    //_debug_uart->integer_print((UInteger)left_motor_encoder.output);
    //_debug_uart->string_print((Text)" r=");
    //_debug_uart->integer_print((UInteger)right_motor_encoder.output);
    //_debug_uart->string_print((Text)"\r\n");

    //_debug_uart->string_print((Text)"c");
    Byte left_speed = (Byte)_left_motor_encoder->output_get();
    Byte right_speed = (Byte)_right_motor_encoder->output_get();

    //_debug_uart->print((Text)"ls=");
    //_debug_uart->integer_print((Integer)left_speed);
    //_debug_uart->print((Text)" le=");
    //_debug_uart->integer_print(left_encoder);
    //_debug_uart->print((Text)" ple=");
    //_debug_uart->integer_print(previous_left_encoder);
    //_debug_uart->print((Text)"\r\n");
    //previous_left_encoder = left_encoder;

    //_debug_uart->string_print((Text)"d");
    if (left_speed != last_left_speed) {
      _left_motor_encoder->pwm_set(left_speed);
      last_left_speed = left_speed;
    } 

    //_debug_uart->string_print((Text)"e");
    if (right_speed != last_right_speed) {
      _right_motor_encoder->pwm_set(right_speed);
      last_right_speed = right_speed;
    }

    //_debug_uart->string_print((Text)"f");
    _is_moving = (Logical)(left_speed != 0) || (right_speed != 0);

    //motor_speeds_set((Byte)left_motor_encoder.output, (Byte)right_motor_encoder.output);
  } else {
    //_debug_uart->string_print((Text)"-");

    // If we're not moving there is nothing more to do:
    // Reset PIDs once, to prevent startup spikes, see
    //    http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
    // PrevInput is considered a good proxy to detect
    // whether reset has already happened

    //_debug_uart->string_print((Text)"g");
    if (!_left_motor_encoder->is_reset() || !_right_motor_encoder->is_reset()) {
      _left_motor_encoder->reset();
      _right_motor_encoder->reset();
    }
  }
  //_debug_uart->string_print((Text)"]");
}

void Bridge::host_to_bus() {
  Frame_Buffer			bus_frame_in;
  Frame_Buffer			bus_frame_out;
  Frame_Buffer			bus_frame_pending;
  UShort 			echo_suppress = 0xfefe;
  UShort			high_bits = 0xfefe;
  Frame_Buffer			host_frame_in;
  Frame_Buffer			host_frame_out;

  //led_set((Logical)1);
  //delay(200);
  //led_set((Logical)0);

  // Just keep looping:
  while (1) {
    // Do we have an 8-bit byte to read from the host?:
    if (_host_uart->can_receive() && !host_frame_in.is_full()) {
      // Receive 8-bit byte and save it into *host_frame_in*:
      UShort frame = _host_uart->frame_get();
      host_frame_in.append(frame);
    }

    // Do we have an 8-bit byte to send to the host?:
    if (!host_frame_out.is_empty() && _host_uart->can_transmit()) {
      // Send *frame* up to host:
      UShort frame = host_frame_out.lop();
      _host_uart->frame_put(frame);
    }

    // Do we have a 9-bit byte to read from the bus?:
    if (_bus_slave->can_receive() && !bus_frame_in.is_full()) {
      // Recevie a bus *frame* and save it into *bus_frame_in*:
      UShort frame = _bus_slave->frame_get();
      bus_frame_in.append(frame);
    }

    // Is there a pending 9-bit frame to send to bus?:
    if (!bus_frame_out.is_empty() && _bus_slave->can_transmit()) {
      // Send *frame* out to the bus:
      UShort frame = bus_frame_out.lop();
      _bus_slave->frame_put(frame);

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
	  //led_blink(200, 800);
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
	  //led_blink(500, 500);
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
	    //led_set((Logical)0);
	    //digitalWrite(LED, LOW);
	    break;
	  case 7:
	    // Reserved:
	    //led_set((Logical)1);
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

void Bridge::motor_speeds_set(Short left_speed, Short right_speed) {
  _left_motor_encoder->pwm_set((Byte)left_speed);
  _right_motor_encoder->pwm_set((Byte)right_speed);
}

void Bridge::setup(UByte test) {
  // Initialize *avr_uart0* as a debugging port:
  _host_uart->begin(16000000L, 115200L, (Character *)"8N1");

  // For debugging, dump out UART0 configuration registers:
  if (system_debug_flags_get() & DBG_FLAG_UART_SETUP) {
    avr_uart0.string_print((Character *)" A:");
    avr_uart0.uinteger_print((UInteger)UCSR0A);
    avr_uart0.string_print((Character *)" B:");
    avr_uart0.uinteger_print((UInteger)UCSR0B);
    avr_uart0.string_print((Character *)" C:");
    avr_uart0.uinteger_print((UInteger)UCSR0C);
    avr_uart0.string_print((Character *)" H:");
    avr_uart0.uinteger_print((UInteger)UBRR0H);
    avr_uart0.string_print((Character *)" L:");
    avr_uart0.uinteger_print((UInteger)UBRR0L);
    avr_uart0.string_print((Character *)"\r\n");
  }

  // Turn the *LED* on:
  //pinMode(LED, OUTPUT);
  //digitalWrite(LED, HIGH);

  // Initalize *avr_uart1* to talk to the bus:
  _bus_uart->begin(16000000L, 500000L, (Character *)"9N1");

  // Force the standby pin on the CAN transeciever to *LOW* to force it
  // into active mode:

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
      _host_uart->string_print((Character *)"\r\nbb_output:\r\n");
      _host_uart->interrupt_set((Logical)0);
      _bus_uart->interrupt_set((Logical)0);
      break;
    case TEST_BUS_ECHO:
      _host_uart->string_print((Character *)"\r\nbb_echo:\r\n");
      _host_uart->interrupt_set((Logical)0);
      _bus_uart->interrupt_set((Logical)0);
      break;
    case TEST_BUS_COMMAND:
      _host_uart->string_print((Character *)"\r\nbb_command:\r\n");
      _host_uart->interrupt_set((Logical)1);
      _bus_uart->interrupt_set((Logical)1);
      break;
    case TEST_BUS_BRIDGE:
      // No announce because we are talking to *host_uart*:
      //host_uart->string_print((Character *)"\r\nbb_bridge:\r\n");
      _host_uart->interrupt_set((Logical)1);
      _bus_uart->interrupt_set((Logical)1);
      break;
    case TEST_RAB_FREYA:
    case TEST_RAB_LOKI:
      // No announce because we are talking to *host_uart*:
      _host_uart->interrupt_set((Logical)1);
      _bus_uart->interrupt_set((Logical)1);
      //_host_uart->string_print(
      //  (Character *)"\r\nros_ard_bridge_protocol:\r\n");
      break;
  }
}

void Bridge::loop(UByte mode) {
  switch (mode) {
    case TEST_RAB_FREYA:
    case TEST_RAB_LOKI: {
      // Some constants:
      static const UInteger PID_RATE = 25;			// Hz.
      static const UInteger PID_INTERVAL = 1000 / PID_RATE;	// mSec.
      static const UInteger MAXIMUM_ARGUMENTS = 4;
      //static const UInteger AUTO_STOP_INTERVAL = 2000;	// mSec.

      // Some variables that need to be unchanged through each loop iteration:
      static Integer arguments[MAXIMUM_ARGUMENTS];
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
      if (_host_uart->can_receive()) {
	// Grab the next character since we have it:
	Character character = (Character)_host_uart->frame_get();

	// Echo the input:  (for terminal but DON'T use for ROS Arduino Bridge
        if (system_debug_flags_get() & DBG_FLAG_ECHO_INPUT_CHARS) {
	  _host_uart->frame_put((UShort)character);
	  if (character == '\r') {
	    _host_uart->frame_put((UShort)'\n');
	  }
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
	  //host_uart->string_print((Text)"<");
	  // Dispatch on *command* character:
	  switch (command) {
	    case 'b': {
	      // Print out baud rate ("b"):
	      _host_uart->string_print((Text)"115200\r\n");
	      break;
	    }
	    case 'e': {
	      // Read encoders ("e"):
	      Integer left_encoder = _left_motor_encoder->encoder_get() *
	       ENCODER_RIGHT_POLARITY;
	      Integer right_encoder = _right_motor_encoder->encoder_get() *
	       ENCODER_LEFT_POLARITY;

	      // Send the results back:
	      _host_uart->integer_print(left_encoder);
	      _host_uart->string_print((Text)" ");
	      _host_uart->integer_print(right_encoder);
	      _host_uart->string_print((Text)"\r\n");
	      break;
	    }
	    case 'm': {
	      // Set motor speeds ("m left, right"):
	      Integer left_speed = arguments[0];
	      Integer right_speed = arguments[1];
	      
	      // For PID code:
	      _is_moving = (Logical)(left_speed != 0 || right_speed != 0);
	      if (_is_moving) {
		_left_motor_encoder->target_ticks_per_frame_set(left_speed);
		_right_motor_encoder->target_ticks_per_frame_set(right_speed);

                if ((system_debug_flags_get() & DBG_FLAG_PID_DISABLE_OK) &&
                // Do normal 'm' command unless Kp are zero, then do direct pwm set
	           (_left_motor_encoder->proportional_get()  == 0) && 
	           (_right_motor_encoder->proportional_get() == 0))  {
	            // bypass PID code if Kp are all zero and scale pwm based on 10 as top speed
                    int motLeftSpeed;
                    int motRightSpeed;
                    if (left_speed > 0) {
                      motLeftSpeed = (left_speed*PID_OVERRIDE_FACTOR) + PID_OVERRIDE_OFFSET;
                      if (motLeftSpeed > 126)  motLeftSpeed = 126;
                    } else {
                      motLeftSpeed = (left_speed*PID_OVERRIDE_FACTOR) - PID_OVERRIDE_OFFSET;
                      if (motLeftSpeed < -126)  motLeftSpeed = -126;
                    }
                    if (right_speed > 0) {
                      motRightSpeed = (right_speed*PID_OVERRIDE_FACTOR) + PID_OVERRIDE_OFFSET;
                      if (motRightSpeed > 126)  motRightSpeed = 126;
                    } else {
                      motRightSpeed = (right_speed*PID_OVERRIDE_FACTOR) - PID_OVERRIDE_OFFSET;
                      if (motRightSpeed < -126)  motRightSpeed = -126;
                    }
	            _left_motor_encoder->pwm_set(motLeftSpeed);
	            _right_motor_encoder->pwm_set(motRightSpeed);
                 }
	      } else {
		motor_speeds_set(0, 0);
	      }

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'p': {
	      // Read object sensor requested  ("o 5"): 
	      Integer sonarUnit = arguments[0];

              // If sonar number is 0 read a bunch of them in units of cm
              int distInCm;
              if (sonarUnit == 0) {
                for (int unit = 1; unit <= 16 ; unit++) {
                  distInCm = (int)((usonar_getLastDistInMm(unit)/(float)(10.0)) + (float)(0.5));
	          _host_uart->integer_print((int)distInCm);
	          _host_uart->string_print((Text)" ");
                }
	        _host_uart->string_print((Text)"\r\n");
              } else {
                // Read sensor on Loki platform from cached measurements
                distInCm = (int)((usonar_getLastDistInMm(sonarUnit)/(float)(10.0)) + (float)(0.5));
	        _host_uart->integer_print((int)distInCm);
	        _host_uart->string_print((Text)"\r\n");
              }

	      break;
	    }
	    case 'o': {      // read a sensor directly inline in units of mm
              // ROS reading of 'pins' or sensors.  ("o 3"):
              // We will return ultrasonic sensor readings for one unit 
              // from the background sampled array of data measured earlier
	      Integer sonarUnit = arguments[0];

              // Read sensor on Loki platform
              float distInMm = usonar_inlineReadMeters(sonarUnit) * 1000.0;
	      _host_uart->integer_print((int)distInMm);
	      _host_uart->string_print((Text)" mm\r\n");

	      break;
	    }
	    case 'r': {
	      // Reset encoders ("r"):
	      _left_motor_encoder->encoder_set(0);
	      _right_motor_encoder->encoder_set(0);

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'u': {
	      // Update PID constants ("U Kp Kd Ki Ci Ko");
	      _left_motor_encoder->proportional_set(arguments[0]);
	      _left_motor_encoder->derivative_set(arguments[1]);
	      _left_motor_encoder->integral_set(arguments[2]);
	      _left_motor_encoder->integral_cap_set(arguments[3]);
	      _left_motor_encoder->denominator_set(arguments[4]);
	      _right_motor_encoder->proportional_set(arguments[0]);
	      _right_motor_encoder->derivative_set(arguments[1]);
	      _right_motor_encoder->integral_set(arguments[2]);
	      _right_motor_encoder->integral_cap_set(arguments[3]);
	      _right_motor_encoder->denominator_set(arguments[4]);
	      _host_uart->string_print((Text)"OK\r\n");

	      // For debugging:
              if (system_debug_flags_get() & DBG_FLAG_PARAMETER_SETUP) {
	        _host_uart->string_print((Text)"Kp ");
	        _debug_uart->integer_print(
	        _left_motor_encoder->proportional_get());
	        _host_uart->string_print((Text)"  Kd ");
	        _debug_uart->integer_print(_left_motor_encoder->derivative_get());
	        _host_uart->string_print((Text)"  Ki ");
	        _debug_uart->integer_print(_left_motor_encoder->integral_get());
	        _host_uart->string_print((Text)"  Ci ");
	        _debug_uart->integer_print(_left_motor_encoder->integral_cap_get());
	        _host_uart->string_print((Text)"  Do ");
	        _debug_uart->integer_print( _left_motor_encoder->denominator_get());
	        _host_uart->string_print((Text)"\r\n");
              }

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'v': {
              // Set verbosity of debug messages  ("v 5"):
	      Integer debug_flags = arguments[0];

              // set the bits in the system wide debug flags
              system_debug_flags_set(debug_flags);

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'z': {
	      // Set motor speeds ("z left right"):
	      Integer left_speed = arguments[0];
	      Integer right_speed = arguments[1];
	      
              // Cap values to range of 8-bit signed value or we get confusing rollover
              if (left_speed > 127) {
                left_speed = 127;
              } else if (left_speed < -127) {
                left_speed = -127;
              }
              if (right_speed > 127) {
                right_speed = 127;
              } else if (right_speed < -127) {
                right_speed = -127;
              }

	      // For PID code:
	      _left_motor_encoder->pwm_set(left_speed);
	      _right_motor_encoder->pwm_set(right_speed);

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    default: {
	      _host_uart->string_print((Text)"Invalid Command!\r\n");
	      break;
	    }
	  }
	  command = '?';

	  // Reset the *arguments_index* for the next command:
	  arguments_index = 0;
	  //host_uart->string_print((Text)">\r\n");
	}
      }

      // Do we need to do a PID update cycle?:
      if (now > next_pid) {
	pid_update(mode);
        next_pid += PID_INTERVAL;
      }

      // Do we need to shut down the motor?:
      //if (now - last_motor_command_time > AUTO_STOP_INTERVAL) {
	//motor_speeds_set(0, 0);
      //}
      break;
    }
    case TEST_BUS_BRIDGE: {
      host_to_bus();
      break;
    }
    case TEST_BUS_COMMAND: {
      // Blink the *LED* some:

      // This code is kind of old...

      // Set the *LED* to *HIGH* and then wait a little:
      //_bus_slave->command_ubyte_put(ADDRESS, LED_PUT, HIGH);
      //Logical led_get = _bus_slave->command_ubyte_get(ADDRESS, LED_GET);
      //led_set(led_get);
      //delay(100);

      // Set the *LED* to *LOW* and then wait a little:
      //_bus_slave->command_ubyte_put(ADDRESS, LED_PUT, LOW);
      //led_get = _bus_slave->command_ubyte_get(ADDRESS, LED_GET);
      //led_set(led_get);
      //delay(100);
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
      _bus_slave->frame_put((UShort)character);

      // Get the resulting *echo_frame* and indicate when it does not match:
      UShort echo_frame = _bus_slave->frame_get();
      if ((UShort)character != echo_frame) {
	_host_uart->string_print((Character *)"!");
      }

      // Wait for the result from the remote module:
      UShort remote_frame = _bus_slave->frame_get();

      // Print the *remote_frame* out to *host_uart*:
      _host_uart->frame_put(remote_frame);

      // Print out any needed CRLF and update to next *character*:
      if (remote_frame == (UShort)'_') {
	_host_uart->string_print((Character *)"\r\n");
	character = '@';
      } else {
	character += 1;
      }

      // Let's blink the LED for a little:
      //led_set((remote_frame & 1) == 0);
      //delay(100);

      //led_set((remote_frame & 1) != 0);
      //delay(100);

      break;
    }
    case TEST_BUS_OUTPUT: {
      // This verision of loop simply outputs 8-bit characters (in 9-bit mode)
      // to the bus starting from '@' to '_' and repeating.  The primary purpose
      // is to verify that both UART's are properly initialized to reasonable
      // baud rates.  We also ensure that the bus is terminated and the
      // CAN bus transceiver is on.

      //UShort delay_milliseconds = 10;

      // *character* is a static variable:
      static Character character = 'U';

      // Make sure *character* is "in bounds":
      if (character < '@' || character > '_') {
	character = '@';
      }

      // Output *character* to bus:
      _bus_slave->frame_put((UShort)character);

      // Set LED to be the same as the LSB of *frame*:
      //led_set((character & 1) != 0);
      //delay(delay_milliseconds);

      // Output *frame* back to user for debugging:
      _debug_uart->frame_put((UShort)character);
      if (character >= '_') {
	// For debugging, dump out UART1 configuration registers:
	//_debug_uart->string_print((Character *)" A:");
	//_debug_uart->uinteger_print((UInteger)UCSR1A);
	//_debug_uart->string_print((Character *)" B:");
	//_debug_uart->uinteger_print((UInteger)UCSR1B);
	//_debug_uart->string_print((Character *)" C:");
	//_debug_uart->uinteger_print((UInteger)UCSR1C);
	//_debug_uart->string_print((Character *)" H:");
	//_debug_uart->uinteger_print((UInteger)UBRR1H);
	//_debug_uart->string_print((Character *)" L:");
	//_debug_uart->uinteger_print((UInteger)UBRR1L);
	//_debug_uart->string_print((Character *)"\r\n");

	_debug_uart->string_print((Character *)"\r\n");
        character = '@';
      } else {
	//character += 1;
      }

      // Set LED to be the opposite of the *frame* LSB:
      //led_set((character & 1) == 0);
      //delay(delay_milliseconds);

      break;
    }
  }
}

