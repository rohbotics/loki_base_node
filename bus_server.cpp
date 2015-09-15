// Copyright (c) 2014-2015 by Wayne C. Gramlich.  All rights reserved.
// Copyright (c) 2015 by Mark Johnston.  All rights reserved.

// http://brettbeauregard.com/blog/2011/04/
//  improving-the-beginner%E2%80%99s-pid-initialization/

#include "Bus_Slave.h"
#include "Frame_Buffer.h"
#include "bus_server.h"
#include "RAB_Sonar.h"

#define PID_OVERRIDE_FACTOR 	 (15)   // Factor used if in PID override mode
#define PID_OVERRIDE_OFFSET      (30)   // Offset to add AFTER PID scaling

// *Bridge* methods:

Bridge::Bridge(AVR_UART *host_uart, AVR_UART *bus_uart, AVR_UART *debug_uart,
 Bus_Slave *bus_slave, Bus_Motor_Encoder *left_motor_encoder,
 Bus_Motor_Encoder *right_motor_encoder, RAB_Sonar *rab_sonar) {

  rab_sonar_ = rab_sonar;
  time_base_ = 0;
  previous_left_encoder_ = 0;
  previous_right_encoder_ = 0;

  // FIXME: These should all have a trailing underscore, not a preceeding one:
  _left_motor_encoder = left_motor_encoder;
  _right_motor_encoder = right_motor_encoder;
  _bus_slave = bus_slave;
  _host_uart = host_uart;
  _bus_uart = bus_uart;
  _debug_uart = debug_uart;
  _is_moving = (Logical)0;
}

// Simple print to debug utility taken out to keep code fairly clean
void Bridge::pid_debug_print_1(Logical verbose)
{
    #ifdef BUS_LOKI_UART1_AS_DEBUG
    _bus_uart->print((Text)"Pid R: ");

    // Print minimal unless in serious debug mode as it takes time
    if (verbose) {
      _bus_uart->print((Text)" Kp ");
      _bus_uart->integer_print(
	(Integer)_right_motor_encoder->proportional_get());
      _bus_uart->print((Text)" Ki ");
      _bus_uart->integer_print((Integer)_right_motor_encoder->integral_get());
      _bus_uart->print((Text)" Kd ");
      _bus_uart->integer_print((Integer)_right_motor_encoder->derivative_get());
      _bus_uart->print((Text)" sPwm ");
      _bus_uart->integer_print((Integer)_right_motor_encoder->output_get());
      _bus_uart->print((Text)" pPwm ");
      _bus_uart->integer_print(
	(Integer)_right_motor_encoder->previous_pwm_get());
      _bus_uart->print((Text)" enc ");
      _bus_uart->integer_print((Integer)_right_motor_encoder->encoder_get());
      _bus_uart->print((Text)" pEnc ");
      _bus_uart->integer_print(
       (Integer)_right_motor_encoder->previous_encoder_get());
      _bus_uart->print((Text)" pIT ");
      _bus_uart->integer_print(
       (Integer)_right_motor_encoder->integral_term_get());
    }

    _bus_uart->print((Text)" tgt ");
    _bus_uart->integer_print(
       (Integer)_right_motor_encoder->target_ticks_per_frame_get());
    #endif
}


// Simple print to debug utility taken out to keep code fairly clean
void Bridge::pid_debug_print_2()
{
    #ifdef BUS_LOKI_UART1_AS_DEBUG
    _bus_uart->print((Text)" Pwm ");
    _bus_uart->integer_print((Integer)_right_motor_encoder->output_get());
    _bus_uart->print((Text)" Perr ");
    _bus_uart->integer_print((Integer)_right_motor_encoder->perr_get());
    _bus_uart->print((Text)" dP ");
    _bus_uart->integer_print((Integer)_right_motor_encoder->pid_delta_get());
    _bus_uart->print((Text)" Rate ");
    _bus_uart->integer_print((Integer)_right_motor_encoder->rate_get());
    _bus_uart->print((Text)" fIT ");
    _bus_uart->integer_print(
     (Integer)_right_motor_encoder->integral_term_get());
    _bus_uart->print((Text)"\r\n");
    #endif
}

void Bridge::pid_update(UByte mode) {
  //_debug_uart->string_print((Text)"[");
  static Byte last_left_speed = 0x80;
  static Byte last_right_speed = 0x80;
  //static Integer previous_left_encoder = 0;

  // Get current encoder values for purpose of Pid loop
  Integer left_encoder = _left_motor_encoder->encoder_get();
  Integer right_encoder = _right_motor_encoder->encoder_get();
  _left_motor_encoder->pid_encoder_set(left_encoder);
  _right_motor_encoder->pid_encoder_set(right_encoder);


  if (_is_moving) {
    //_debug_uart->string_print((Text)"a");

    // Do the PID for each motor:
    //debug_uart->string_print((Text)"b");

    if (rab_sonar_->debug_flags_get() & DBG_FLAG_PID_DEBUG) {
      _host_uart->string_print((Text)"P\r\n");
    }

    pid_debug_print_1((Logical)(0));    // Debug to bus_uart if enabled

    _left_motor_encoder->do_pid();
    _right_motor_encoder->do_pid();

    pid_debug_print_2();                // Debug to bus_uart if enabled

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

    //motor_speeds_set((Byte)left_motor_encoder.output,
    // (Byte)right_motor_encoder.output);
  } else {
    //_debug_uart->string_print((Text)"-");

    // If we're not moving there is nothing more to do:
    // Reset PIDs once, to prevent startup spikes, see
    // PrevInput is considered a good proxy to detect
    // whether reset has already happened

    //_debug_uart->string_print((Text)"g");
    if (!_left_motor_encoder->is_reset() || !_right_motor_encoder->is_reset()) {
      _left_motor_encoder->reset();
      _right_motor_encoder->reset();
    }
  }

  // Save prior encoder settings for the next update
  _left_motor_encoder->previous_encoder_set(left_encoder);
  _right_motor_encoder->previous_encoder_set(right_encoder);

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
  if (rab_sonar_->debug_flags_get() & DBG_FLAG_UART_SETUP) {
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

  // Initalize *avr_uart1* to talk to the bus or debug as desired
  #ifdef BUS_LOKI_UART1_AS_DEBUG
  _bus_uart->begin(16000000L, 115200L, (Character *)"8N1");
  #else
  _bus_uart->begin(16000000L, 500000L, (Character *)"9N1");
  #endif

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
      static const UInteger PID_RATE = 50;			// Hz.
      static const UInteger PID_INTERVAL = 1000 / PID_RATE;	// mSec.
      static const UInteger MAXIMUM_ARGUMENTS = 6;
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
        if (rab_sonar_->debug_flags_get() & DBG_FLAG_ECHO_INPUT_CHARS) {
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
	      Integer left_encoder = _left_motor_encoder->encoder_get();
	      Integer right_encoder = _right_motor_encoder->encoder_get();

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

                if ((rab_sonar_->debug_flags_get() & DBG_FLAG_PID_DISABLE_OK) &&
	         (_left_motor_encoder->proportional_get()  == 0) && 
	         (_right_motor_encoder->proportional_get() == 0))  {
                  // Do normal 'm' command unless Kp are zero, then do
		  // direct pwm set bypass PID code if Kp are all zero
		  // and scale pwm based on 10 as top speed
                  int motLeftSpeed;
                  int motRightSpeed;
                  if (left_speed > 0) {
                    motLeftSpeed =
		     (left_speed*PID_OVERRIDE_FACTOR) + PID_OVERRIDE_OFFSET;
                    if (motLeftSpeed > 126) {
		      motLeftSpeed = 126;
		    }
                  } else {
                    motLeftSpeed =
		     (left_speed*PID_OVERRIDE_FACTOR) - PID_OVERRIDE_OFFSET;
		    if (motLeftSpeed < -126) {
		      motLeftSpeed = -126;
		    }
                  }
                  if (right_speed > 0) {
                    motRightSpeed =
		     (right_speed*PID_OVERRIDE_FACTOR) + PID_OVERRIDE_OFFSET;
                    if (motRightSpeed > 126) {
		      motRightSpeed = 126;
		    }
                  } else {
                    motRightSpeed =
		     (right_speed*PID_OVERRIDE_FACTOR) - PID_OVERRIDE_OFFSET;
                    if (motRightSpeed < -126) {
		      motRightSpeed = -126;
		    }
                  }
	          _left_motor_encoder->pwm_set(motLeftSpeed);
	          _right_motor_encoder->pwm_set(motRightSpeed);
                }
	      } else {
		motor_speeds_set(0, 0);
	      }
	      // Compute *direction* with which corresponds to the
	      // the sum of *left_speed* and *right_speed* but reduced
	      // down to a single signed byte:
	      if (left_speed > 63) {
		left_speed = 63;
	      } else if (left_speed < -63) {
		left_speed = -63;
	      }
 	      if (right_speed > 63) {
		right_speed = 63;
	      } else if (right_speed < -63) {
		right_speed = -63;
	      }
	      Byte direction = (Byte)left_speed + (Byte)right_speed;
	      rab_sonar_->direction_set(direction);

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'o': {      // read a sensor directly inline in units of mm
              // ROS reading of 'pins' or sensors.  ("o 3"):
              // We will return ultrasonic sensor readings for one unit 
              // from the background sampled array of data measured earlier
	      UByte sonarUnit = (UByte)arguments[0];

              // Read sensor on Loki platform
              Short distInMm = rab_sonar_->ping_get(sonarUnit);
	      _host_uart->integer_print((int)distInMm);
	      _host_uart->string_print((Text)" mm\r\n");

	      break;
	    }
	    case 'p': {
	      // Read (ping) sonar requested  ("p 5"): 
	      Integer sonar_unit = arguments[0];

              // If sonar number is 0 read a bunch of them in units of cm
	      UByte sonars_count = rab_sonar_->sonars_count_get();
              if (sonar_unit < sonars_count) {
                // Read sensor on Loki platform from cached measurements
                UShort distance = rab_sonar_->ping_get(sonar_unit);
	        _host_uart->integer_print((Integer)distance);
	        _host_uart->string_print((Text)"\r\n");
              } else {
                for (UByte unit = 0; unit < sonars_count ; unit++) {
                  UShort distance = rab_sonar_->ping_get(unit);
	          _host_uart->integer_print((Integer)distance);
	          _host_uart->string_print((Text)" ");
                }
	        _host_uart->string_print((Text)"\r\n");
              }
	      break;
	    }
	    case 'q': {
	      // Flush the sensor queue.  The format of the response is
	      //
	      //   delta_time_base sensor1 sensor2 ... sensorN
	      //
	      // where:
	      //
	      // * *delta_time_base* is the change in time_base from the
	      //   the previous *time_base* measured in microseconds.
	      //   All sensor times are relative to the new *time_base*.
	      //
	      // * *SensorI* is a sensor triple of the form "id:time:value"
	      //   where *id* is a number that identifies the sensor, *time*
	      //   is a signed time in microseconds relative to *time_base*,
	      //   and *value* is a signed value for the sensor.
	      //
	      // For now, the sensor are hard coded as:
	      //
	      // * 0: left encoder    # Value is a delta from the prevous value
	      // * 1: right encoder   # Value is a delta from the previous value
	      // * 2: Sonar 0         # Value is the distance in microseconds:
	      // * 3: Sonar 1         # Value is the distance in microseconds:
	      //   ...
	      // * 17: Sonar 15       # Value is the distance in microseconds:

	      // Send the increase in *time_base_* and update it as well:
	      UInteger next_time_base = micros();
	      _host_uart->integer_print((Integer)(next_time_base - time_base_));
	      time_base_ = next_time_base;

	      // Grab the left/right encoder values:
	      UInteger left_encoder = _left_motor_encoder->encoder_get();
	      UInteger right_encoder = _right_motor_encoder->encoder_get();

	      //_host_uart->string_print((Text)" [");
	      //_host_uart->integer_print((Integer)left_encoder);
	      //_host_uart->string_print((Text)":");
	      //_host_uart->integer_print((Integer)right_encoder);
	      //_host_uart->string_print((Text)"]");

	      // Send left *left encoder* delta if it changed:
	      if (left_encoder != previous_left_encoder_) {
		_host_uart->string_print((Text)" 0:0:");
		_host_uart->integer_print(
		  (Integer)(left_encoder - previous_left_encoder_));
		previous_left_encoder_ = left_encoder;
	      }
	      
	      // Send the *right_encoder* delta if it changed:
	      if (right_encoder != previous_right_encoder_) {
		_host_uart->string_print((Text)" 1:0:");
		_host_uart->integer_print(
		  (Integer)(right_encoder - previous_right_encoder_));
		previous_right_encoder_ = right_encoder;
	      }

	      // Sonar queue responses go here:
	      rab_sonar_->queue_poll(_host_uart, time_base_, 2);

	      // Terminate the response:
	      _host_uart->string_print((Text)"\r\n");
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
	    case 's': {
	      // Sonar configure ("s sonar_id class left_id right_id"):
	      
	      if (arguments_index == 3) {
		UByte sonar_id = arguments[0];
		Sonar_Class sonar_class = (Sonar_Class)arguments[1];
		Byte left_id = arguments[2];
		Byte right_id = arguments[3];
		rab_sonar_->configure(sonar_id, sonar_class, left_id, right_id);
		_host_uart->string_print((Text)"OK\r\n");
	      } else {
		_host_uart->string_print((Text)"Bad s command\r\n");
	      }
	      break;
	    }
	    case 'u': {
	      // Update or just show PID constants ("u Kp Kd Ki Ko Ci");

              if (arguments_index == 5) {
	        _left_motor_encoder->proportional_set(arguments[0]);
	        _left_motor_encoder->derivative_set(arguments[1]);
	        _left_motor_encoder->integral_set(arguments[2]);
	        _left_motor_encoder->denominator_set(arguments[3]);
	        _left_motor_encoder->integral_cap_set(arguments[4]);

	        _right_motor_encoder->proportional_set(arguments[0]);
	        _right_motor_encoder->derivative_set(arguments[1]);
	        _right_motor_encoder->integral_set(arguments[2]);
	        _right_motor_encoder->denominator_set(arguments[3]);
	        _right_motor_encoder->integral_cap_set(arguments[4]);
	        _host_uart->string_print((Text)"OK\r\n");
              }

	      // For debugging:
              if ((arguments_index < 5) ||
                  (rab_sonar_->debug_flags_get() & DBG_FLAG_PARAMETER_SETUP)) {
	        _host_uart->string_print((Text)"Kp ");
	        _debug_uart->integer_print(
		 _left_motor_encoder->proportional_get());
	        _host_uart->string_print((Text)"  Kd ");
	        _debug_uart->integer_print(
		 _left_motor_encoder->derivative_get());
	        _host_uart->string_print((Text)"  Ki ");
	        _debug_uart->integer_print(
		 _left_motor_encoder->integral_get());
	        _host_uart->string_print((Text)"  Ko ");
	        _debug_uart->integer_print(
		 _left_motor_encoder->denominator_get());
	        _host_uart->string_print((Text)"  Ci ");
	        _debug_uart->integer_print(
		 _left_motor_encoder->integral_cap_get());
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
              rab_sonar_->debug_flags_set(debug_flags);

	      // Print the usual "OK" result:
	      _host_uart->string_print((Text)"OK\r\n");
	      break;
	    }
	    case 'z': {
	      // Set motor speeds ("z left right"):
	      Integer left_speed = arguments[0];
	      Integer right_speed = arguments[1];
	      
              // Cap values to range of 8-bit signed value or we get
	      // confusing rollover:
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

