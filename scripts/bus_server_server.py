#!/usr/bin/env python

import thread
import sys
import glob
import time
import rospy
# Use PYTHONPATH env. var to find location of bus_server.srv directory:
from bus_server.srv import *
from serial import *

## @class Bus_Base
#
# Provide the shared interface to the MakerBus.
#
# *Bus_Base* provides an interface to the serial port that
# talks the 8-bit to 9-bit protocol needed to communicate packets
# back and forth from the Makerbus

class Bus_Base:

    def __init__(self, serial_name):
	""" Bus_Base: Initialize a *Bus_Base* object. """

	#serial_name = None
	if not isinstance(serial_name, str):
	    # Search command line arguments for pattern "/dev/tty*":
	    argv_serials = []
	    for arg in sys.argv:
		if arg.find("/dev/tty") == 0:
		    # Found one:
		    argv_serials.append(arg)
	    unix_serials = glob.glob("/dev/ttyUSB*")
	    macos_serials = glob.glob("/dev/tty.usbserial-*")
	    pi_serials = glob.glob("/dev/ttyAMA*")

	    # Sort everything that we found and concatente them together:
	    argv_serials.sort()
	    pi_serials.sort()
	    unix_serials.sort()
	    macos_serials.sort()
	    serials = argv_serials + unix_serials + macos_serials + pi_serials
	    print("serials={0}".format(serials))

	    # Squirt out an error message
	    if len(serials) == 0:
		print("There is no serial port to open.")
		serial_name = None
	    else:
		# Use the device listed on the command line:
		serial_name = serials[0]

	# Try to open a serial connection:
	serial = None
	if serial_name != None:
	    try:
		serial = Serial(serial_name, 115200)
	    except SerialException:
		serial = None
	if serial == None:
	    print("Unable to open serial port '{0}'".format(serial_name))

	self.address = -1
	self.auto_flush = True
	self.request = []
	self.request_safe = 0
	self.response = []
	self.same_address_requests = 0
	self.serial = serial
	self.trace = False
	self.trace_pad = ""
	self.mutex = thread.allocate_lock()

	#FIXME: Only open serial if it is not already open:
	#serial.open()
	if serial != None:
	    serial.flushInput()
	    serial.setTimeout(0.004)
	
    def auto_flush_set(self, flush_mode):
	""" Bus_Base: This routine will set the auto flush mode for
	    *self* to *flush_mode*.  When *flush_mode* is set to *True*, it will
	    automatically flush each command sequence as it soon as possible.
	    When *flush_mode* is *False*, the command sequences are queued
	    up until they are explicitly flushed by calling *flush*(). """

	assert isinstance(flush_mode, bool)

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.auto_flush({1})". \
	      format(trace_pad, flush_mode))

	self.auto_flush = flush_mode
	if flush_mode:
	    self.flush()

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.auto_flush({1})".
	      format(trace_pad, flush_mode))

    def bus_test(self):
	""" Bus_Base: Shove out a bunch of frames so that the
	    bus can be examined with an oscilliscope. """

	for count in range(0, 10000):
	    self.frame_put(0x185)
	    self.flush()
	    time.sleep(0.05)

    def bus_reset(self):
	""" {Bus_Base}: Reset the bus. """

	trace = self.trace
	if trace:
	    print("=>bus_reset()")

	# Shove a 0xc5 out there to force a bus reset:
	serial = self.serial
	serial.write(chr(0xc5))
	serial.flush()

	# Wait for a response:
	byte = serial.read(1)
	if len(byte) == 0:
	    print("Bus reset failed with no response")
	elif ord(byte) != 0xa5:
	    print("Bus reset failed 0x{0:x}".format(ord(byte)))
	else:
	    if trace:
		print("Bus reset succeeded")

	if trace:
	    print("<=bus_reset()")

    def discovery_mode(self):
	""" {Bus_Base}: Perform discovery mode """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "


	serial = self.serial
	serial.write(chr(0xc4))
	if trace:
	    print("{0}write(0xc4)".format(trace_pad))

	serial.flush()
	line = []
	ids = []
	done = False
	while not done:
	    byte = serial.read(1)

	    if trace:
		print("{0}read() => 0x{1:x}".format(trace_pad, ord(byte)))

	    if byte == '\n':
		ids.append("".join(line[1:]))
		done = len(line) != 0 and line[0] == '!'
		del line[:]
	    else:
		line.append(byte)

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.discovery_mode() =>{1}". \
	      format(trace_pad, ids))

	return ids

    def flush(self):
	""" Bus_Base: Flush out current request. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.flush()".format(trace_pad))

	# Do not do anything unless we have something to send:
	request = self.request
	request_length = len(request)
	while request_length != 0:
	    # Make sure that the correct module is selected:
	    if request_length >= 16:
		request_length = self.request_safe
	    assert request_length < 16, \
	      "Request is {0} bytes >= 16".format(request_length)

	    # Compute checksum:
	    checksum = 0
	    for index in range(0, request_length):
		checksum += request[index]
	    checksum = (checksum + (checksum >> 4)) & 0xf

	    # Compute request header and send it out:
	    request_header = (request_length << 4) | checksum
	    serial = self.serial
	    self.frame_put(request_header)
	
	    # Send out the rest of the request:
	    for index in range(0, request_length):
		ubyte = request[index]
		self.frame_put(ubyte)
	    del request[0: request_length]
	    request_length = len(request)
	    self.request_safe -= request_length

	    # Flush the serial output buffer:
	    if trace:
		print("{0}Bus.flush:serial.flush()".format(trace_pad))
	    serial.flush()

	    # Now get a response:
	    response = self.response
	    response_header = self.frame_get()
	    if response_header < 0:
		# We had a time-out:
		print("Response header timeout")
	    else:
		response_length = response_header >> 4
		response_checksum = response_header & 0xf
	
		# Get the rest of the response:
		del response[:]
		while response_length != 0:
		    response_frame = self.frame_get()
		    if response_frame < 0:
			# We have a timeout:
			print("Response byte timeout")
			break
		    response.append(response_frame)
		    response_length -= 1

		# Compute checksum:
		checksum = 0
		for ubyte in response:
		    checksum += ubyte
		checksum = (checksum + (checksum >>4)) & 0xf

		if trace:
		    print("{0}response={1}, checksum=0x{2:x}". \
		      format(trace_pad, response, checksum))

		if checksum != response_checksum:
		    print("Got checksum of 0x{0:x} instead of 0x{1:x}". \
		      format(checksum, response_checksum))
		    del response[:]

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.flush() response={1}". \
	      format(trace_pad, self.response))

    def frame_get(self):
	""" {Bus_Base}: Return the next frame from the bus connected
	    to {self}. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.frame_get()".format(trace_pad))

	frame = -10;
	serial = self.serial
	result = serial.read(1)
	if len(result) != 0:
	    frame = ord(result[0])
	else:
	    print("Timeout1")
	    frame = -1
	    self.address = -1

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.frame_get()=>0x{1:x}". \
	      format(trace_pad, frame))
	return frame

    def frame_put(self, frame):
	""" {Bus_Base}: Send frame to the bus connected to {self}. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.frame_put(0x{1:x})".format(trace_pad, frame))

	serial = self.serial
	if (frame > 0xff or (0xc1 <= frame and frame <= 0xc5)):
	    # Send {frame} as two bytes:
	    byte1 = 0xc0 | ((frame >> 7) & 3)
	    byte2 = frame & 0x7f
	    serial.write(chr(byte1))
	    serial.write(chr(byte2))

	    if trace:
		print("{0}write(0x{1:x});write(0x{2:x})". \
		  format(trace_pad, byte1, byte2))
	else:
	    # Send {frame} as one byte:
	    serial.write(chr(frame))

	    if trace:
		print("{0}write(0x{1:x})".format(trace_pad, frame))
	    
	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.frame_put(0x{1:x})".format(trace_pad, frame))


    def request_begin(self, address, command):
	""" {Bus_Base}: Append {command} to self.request. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    print("{0}=>Bus.request_begin(0x{1:x}, 0x{2:x})". \
	      format(trace_pad, address, command))
	    self.trace_pad = trace_pad + " "

	request = self.request
	request_length = len(request)
	self.request_safe = request_length
	if self.auto_flush and request_length != 0:
	    self.flush()

	if address == self.address:
	    if self.same_address_requests >= 8:
		self.address = -1
	    else:
		self.same_address_requests += 1

	if address != self.address:
	    self.same_address_requests = 0
	    self.frame_put(address | 0x100)
	    self.address = address
	    if (address & 0x80) == 0:
		self.serial.flush()
		self.frame_get()

	request.append(command)

	if trace:
	    print("{0}<=Bus.request_begin(0x{1:x}, 0x{2:x})". \
	      format(trace_pad, address, command))
	    self.trace_pad = trace_pad

    def request_byte_put(self, byte):
	""" {Bus_Base}: Append {byte} to current request in {self}. """

	if byte >= 0:
	    ubyte = byte
	else:
	    ubyte = 0xff + byte + 1
	self.request_ubyte_put(self, ubyte);

    def request_end(self):
	""" {Bus_Base}: Indicate that current command is complete. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.request_end()".format(trace_pad))

	request = self.request
	request_length = len(request)
	if request_length >= 16:
	    self.flush()
	self.request_safe = len(request)
	if self.auto_flush:
	    self.flush()

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.request_end() response={1}". \
	      format(trace_pad, self.response))

    def request_integer_put(self, integer):
	""" {Bus_Base}: Append {int32} to current request in {self}. """

	# Python 3.0 has infinite precision *int*'s, so we we have to
	# convert carefully:
	if integer >= 0:
	    uinteger = integer
	else:
	    uinteger = 0xffffffff + integer + 1
	self.request_uinteger_put(uinteger);

    def request_short_put(self, short):
	""" {Bus_Base}: Append {int16} to current request in {self}. """

	# Python 3.0 has infinite precision *int*'s, so we we have to
	# convert carefully:
	if short >= 0:
	    ushort = short
	else:
	    ushort = 0xffff + short + 1
	self.request_ushort_put(ushort);

    def request_ubyte_put(self, ubyte):
	""" {Bus_Base}: Append {ubyte} to current request in {self}. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.request_ubyte_put({1})". \
	      format(trace_pad, ubyte))

	request = self.request
	request.append(ubyte & 0xff)

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.request_ubyte_put({1}) request={2}". \
	      format(trace_pad, ubyte, request))

    def request_uinteger_put(self, uinteger):
	""" {Bus_Base}: Append {uinteger} to current request in
	    {self}. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.request_uinteger_put({1})". \
	      format(trace_pad, uinteger))

	assert isinstance(uinteger, int)
	self.request_ubyte_put((uinteger >> 24) & 0xff)
	self.request_ubyte_put((uinteger >> 16) & 0xff)
	self.request_ubyte_put((uinteger >> 8)  & 0xff)
	self.request_ubyte_put( uinteger	& 0xff)

	if trace:
	    print("{0}<=Bus.request_uinteger_put({1})". \
	      format(trace_pad, uinteger))

    def request_ushort_put(self, ushort):
	""" {Bus_Base}: Append {ushort} to current request in {self}. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.request_ushort_put({1})". \
	      format(trace_pad, ushort))

	assert isinstance(ushort, int)
	self.request_ubyte_put((ushort >> 8) & 0xff)
	self.request_ubyte_put( ushort       & 0xff)

	if trace:
	    self.trace_pad = trace_pad
	    print("{0}<=Bus.request_ushort_put({1})". \
	      format(trace_pad, ushort))

    def response_begin(self):
	""" {Bus_Base}: Begin a response sequence. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.response_begin() response={1}". \
	      format(trace_pad, self.response))

	self.flush()

	if trace:
	    print("{0}<=Bus.response_begin()".format(trace_pad))
	    self.trace_pad = trace_pad

    def response_byte_get(self):
	""" {Bus_Base}: Return next unsigned byte from response
	     in {self}. """

	ubyte = self.response_ubyte_get()
	if ubyte & 0x80 != 0:
	    ubyte |= 0xffffff00
	return ubyte

    def response_end(self):
	""" {Bus_Base}: End a response sequence. """

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.response_end() response={1}". \
	      format(trace_pad, self.response))

	response = self.response
	response_length = len(response)
	assert len(response) == 0, \
	  "{0} bytes left over from response".format(response_length)

	if trace:
	    print("{0}<=Bus.response_end()".format(trace_pad))
	    self.trace_pad = trace_pad


    def response_integer_get(self):
	""" {Bus_Base}: Return next unsigned byte from response
	    in {self}. """

	# Python 3.0 has infinte precesion *int*'s, so we have to
	# carefully convert to negative numbers:
	uinteger = self.response_uinteger_get()
	if (uinteger & 0x8000000) == 0:
	    integer = uinteger
	else:
	    integer = -((uinteger & 0x7fffffff) ^ 0x7fffffff) - 1
	return integer

    def response_byte_get(self):
	""" {Bus_Base}: Return next unsigned byte from response
	    in {self}. """

	# Python 3.0 has infinte precesion *int*'s, so we have to
	# carefully convert to negative numbers:
	ubyte = self.response_ubyte_get()
	if ubyte & 0x80 == 0:
	    byte = ubyte
	else:
	    byte = -((ubyte & 0x7f) ^ 0x7f) - 1
	return byte

    def response_short_get(self):
	""" {Bus_Base}: Return next unsigned byte from response
	    in {self}. """

	# Python 3.0 has infinte precesion *int*'s, so we have to
	# carefully convert to negative numbers:
	ushort= self.response_ushort_get()
	if ushort & 0x8000 == 0:
	    short = ushort
	else:
	    short = -((ushort & 0x7fff) ^ 0x7fff) - 1
	return short

    def response_ubyte_get(self):
	""" {Bus_Base}: Return next unsigned byte from response
	    in {self}. """

	response = self.response

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.response_ubyte_get() response={1}". \
	      format(trace_pad, response))

	ubyte = response[0]
	del response[0]

	if trace:
	    print("{0}<=Bus.response_ubyte_get()=>{1}". \
	      format(trace_pad, ubyte))
	    self.trace_pad = trace_pad

	return ubyte

    def response_uinteger_get(self):
	""" {Bus_Base}: Return next unsigned integer from response in
	    {self}. """

	byte0 = self.response_ubyte_get()
	byte1 = self.response_ubyte_get()
	byte2 = self.response_ubyte_get()
	byte3 = self.response_ubyte_get()
	result = (byte0 << 24) | (byte1 << 16) | (byte2 << 8) | byte3

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.response_uinteger_get() result={1}". \
	      format(trace_pad, result))

	return result

    def response_ushort_get(self):
	""" {Bus_Base}: Return next unsigned short from response
	    in {self}. """

	response = self.response

	trace = self.trace
	if trace:
	    trace_pad = self.trace_pad
	    self.trace_pad = trace_pad + " "
	    print("{0}=>Bus.response_ushort_get() response={1}". \
	      format(trace_pad, response))

	high_ubyte = self.response_ubyte_get()
	low_ubyte = self.response_ubyte_get()
	ushort = (high_ubyte << 8) | low_ubyte

	if trace:
	    print("{0}<=Bus.response_ushort_get()=>{1}". \
	      format(trace_pad, ushort))
	    self.trace_pad = trace_pad

	return ushort

## @class Bus_Module
#
# Per module base class to interface with MakerBus modules.
#
# *Bus_Module* provides a base class from which sub-classes
# can be specialized.  The base class provides common communication
# methods.  The specialized sub-class provides the module specific
# register and function access to the module.

class Bus_Module:
    """ {Bus_Module}: This represents a single module on the bus: """

    def __init__(self, bus_base, address, offset):
	""" {Bus_Module}: Initialize {self} to contain {bus_base}
	    and {address}."""

	assert isinstance(bus_base, Bus_Base)
	assert isinstance(address, int)
	assert isinstance(offset, int)

	self.bus_base = bus_base
	self.address = address
	self.offset = offset

    def auto_flush_set(self, flush_mode):
	""" {Bus_Module}:  This routine will set the auto flush mode for
	    {self} to {flush_mode}.  When {flush_mode} is {True},
	    each operation is immediately sent to the selected module.
	    When {flush_mode} is {False}, the commands queue up until
	    the request buffer is full or until {flush}() is explicitly
	    called. """

	self.bus_base.auto_flush_set(flush_mode)

    def flush(self):
	""" {Bus_Module}: This routine will cause any queued commands
	    to be flushed.  """

	self.bus_base.flush()

    def request_begin(self, command):
	""" {Bus_Module}: """

	self.bus_base.request_begin(self.address, self.offset + command)

    def request_byte_put(self, byte):
	""" {Bus_Module}: """

	self.bus_base.request_ubyte_put(byte)

    def request_character_put(self, character):
	""" {Bus_Module}: """

	self.bus_base.request_ubyte_put(ord(character))

    def request_end(self):
	""" {Bus_Module}: """

	self.bus_base.request_end()

    def request_integer_put(self, int32):
	""" {Bus_Module}: """

	self.bus_base.request_integer_put(int32)

    def request_logical_put(self, logical):
	""" {Bus_Module}: """

	value = 0
	if logical:
	    value = 1
	self.bus_base.request_ubyte_put(value)

    def request_short_put(self, int16):
	""" {Bus_Module}: """

	self.bus_base.request_short_put(int16)

    def request_ubyte_put(self, ubyte):
	""" {Bus_Module}: """

	self.bus_base.request_ubyte_put(ubyte & 0xff)

    def request_uinteger_put(self, uint32):
	""" {Bus_Module}: """

	# High byte first, followed by low byte:
	self.request_uinteger_put(uint32)

    def request_ushort_put(self, uint16):
	""" {Bus_Module}: """

	# High byte first, followed by low byte:
	self.request_ushort_put(uint16)

    def response_begin(self):
	""" {Bus_Module}: """

	self.bus_base.response_begin()

    def response_byte_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_byte_get()

    def response_character_get(self):
	""" {Bus_Module}: """

	return chr(self.bus_base.response_ubyte_get())

    def response_logical_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_ubyte_get() != 0

    def response_byte_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_byte_get()

    def response_short_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_short_get()

    def response_integer_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_integer_get()

    def response_ubyte_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_ubyte_get()

    def response_ushort_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_ushort_get()

    def response_uinteger_get(self):
	""" {Bus_Module}: """

	return self.bus_base.response_uinteger_get()

    def response_end(self):
	""" {Bus_Module}: """

	self.bus_base.response_end()

class Bus_Server:

    def __init__(self):
	""" Bus_Server: Initialize *self*. """

	bus_base = Bus_Base(None)
	self.bus_base = bus_base

    # Below are the call back routines that are invode when a service
    # request comes in.

    def bus_bool_get(self, request):
	""" Bus_Server: Get a boolean. """

	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	ubyte = bus_base.response_ubyte_get()
	bus_base.response_end()
	mutex.release()
	value = False
	if ubyte != 0:
	    value = True
	return BusBoolGetResponse(value, 0)

    def bus_bool_set(self, request):
	""" Bus_Server: Set a boolean. """

	bus_base = self.bus_base
	mutex = bus_base.mutex
	ubyte = 0
	if request.value:
	    ubyte = 1
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_ubyte_put(ubyte)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
	return BusBoolSetResponse(0)

    def bus_int16_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_short_get()
	bus_base.response_end()
	mutex.release()
	return BusInt16GetResponse(value, 0)

    def bus_int16_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_short_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
	return BusInt16SetResponse(0)

    def bus_int32_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_integer_get()
	bus_base.response_end()
	mutex.release()
	return BusInt32GetResponse(value, 0)

    def bus_int32_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_integer_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
        return BusInt32SetResponse(0)

    def bus_int64_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_long_get()
	bus_base.response_end()
	mutex.release()
	return BusInt64GetResponse(value, 0)

    def bus_int64_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_long_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
	return BusInt64SetResponse(0)

    def bus_int8_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_byte_get()
	bus_base.response_end()
	mutex.release()
	return BusInt8GetResponse(value, 0)

    def bus_int8_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_byte_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
        return BusInt8SetResponse(0)

    def bus_uint16_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_ushort_get()
	bus_base.response_end()
	mutex.release()
	return BusUInt16GetResponse(value, 0)

    def bus_uint16_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_ushort_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
	return BusUInt16SetResponse(0)

    def bus_uint32_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_uinteger_get()
	bus_base.response_end()
	mutex.release()
	return BusUInt32GetResponse(value, 0)

    def bus_uint32_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_uinteger_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
        return BusUInt32SetResponse(0)

    def bus_uint64_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_ulong_get()
	bus_base.response_end()
	mutex.release()
	return BusUInt64GetResponse(value, 0)

    def bus_uint64_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_ulong_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
	return BusUInt64SetResponse(0)

    def bus_uint8_get(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_end()
	bus_base.response_begin()
	value = bus_base.response_ubyte_get()
	bus_base.response_end()
	mutex.release()
	return BusUInt8GetResponse(value, 0)

    def bus_uint8_set(self, request):
	bus_base = self.bus_base
	mutex = bus_base.mutex
	mutex.acquire()
	bus_base.request_begin(request.address, request.command)
	bus_base.request_ubyte_put(request.value)
	bus_base.request_end()
	bus_base.flush()
	bus_base.response_begin()
	bus_base.response_end()
	mutex.release()
        return BusUInt8SetResponse(0)

    def run(self):
	# Register the server name:
	rospy.init_node("bus_server_server")

	# Register all of the services:
	rospy.Service("bus_bool_get",   BusBoolGet,   self.bus_bool_get)
	rospy.Service("bus_bool_set",   BusBoolSet,   self.bus_bool_set)
	rospy.Service("bus_int16_get",  BusInt16Get,  self.bus_int16_get)
	rospy.Service("bus_int16_set",  BusInt16Set,  self.bus_int16_set)
	rospy.Service("bus_int32_get",  BusInt32Get,  self.bus_int32_get)
	rospy.Service("bus_int32_set",  BusInt32Set,  self.bus_int32_set)
	rospy.Service("bus_int64_get",  BusInt64Get,  self.bus_int64_get)
	rospy.Service("bus_int64_set",  BusInt64Set,  self.bus_int64_set)
	rospy.Service("bus_int8_get",   BusInt8Get,   self.bus_int8_get)
	rospy.Service("bus_int8_set",   BusInt8Set,   self.bus_int8_set)
	rospy.Service("bus_uint16_get", BusUInt16Get, self.bus_uint16_get)
	rospy.Service("bus_uint16_set", BusUInt16Set, self.bus_uint16_set)
	rospy.Service("bus_uint32_get", BusUInt32Get, self.bus_uint32_get)
	rospy.Service("bus_uint32_set", BusUInt32Set, self.bus_uint32_set)
	rospy.Service("bus_uint64_get", BusUInt64Get, self.bus_uint64_get)
	rospy.Service("bus_uint64_set", BusUInt64Set, self.bus_uint64_set)
	rospy.Service("bus_uint8_get",  BusUInt8Get,  self.bus_uint8_get)
	rospy.Service("bus_uint8_set",  BusUInt8Set,  self.bus_uint8_set)

	# Start the server node:
	print("Starting bus_server_server")
	rospy.spin()

def main():
    """ main: Run the server. """

    bus_server = Bus_Server()
    bus_server.run()

if __name__ == "__main__":
    main()



