#!/usr/bin/env python

import rospy
# Use PYTHONPATH env. var to find location of bus_server.srv directory:
from bus_server.srv import *

registers = {}
registers["bool"] = False
registers["int16"] = -16
registers["int32"] = -32
registers["int64"] = -64
registers["int8"] = -8
registers["uint16"] = 16
registers["uint32"] = 32
registers["uint64"] = 64
registers["uint8"] = 8

def bus_bool_get(request):
    return BusBoolGetResponse(registers["bool"], 0)

def bus_bool_set(request):
    registers["bool"] = request.value
    return BusBoolSetResponse(0)

def bus_int16_get(request):
    return BusInt16GetResponse(registers["int16"], 0)

def bus_int16_set(request):
    registers["int16"] = request.value
    return BusInt16SetResponse(0)

def bus_int32_get(request):
    return BusInt32GetResponse(registers["int32"], 0)

def bus_int32_set(request):
    registers["int32"] = request.value
    return BusInt32SetResponse(0)

def bus_int64_get(request):
    return BusInt64GetResponse(registers["int64"], 0)

def bus_int64_set(request):
    registers["int64"] = request.value
    return BusInt64SetResponse(0)

def bus_int8_get(request):
    return BusInt8GetResponse(registers["int8"], 0)

def bus_int8_set(request):
    registers["int8"] = request.value
    return BusInt8SetResponse(0)

def bus_uint16_get(request):
    return BusUint16GetResponse(registers["uint16"], 0)

def bus_uint16_set(request):
    registers["uint16"] = request.value
    return BusUint16SetResponse(0)

def bus_uint32_get(request):
    return BusUint32GetResponse(registers["uint32"], 0)

def bus_uint32_set(request):
    registers["uint32"] = request.value
    return BusUint32SetResponse(0)

def bus_uint64_get(request):
    return BusUint64GetResponse(registers["uint64"], 0)

def bus_uint64_set(request):
    registers["uint64"] = request.value
    return BusUint64SetResponse(0)

def bus_uint8_get(request):
    return BusUint8GetResponse(registers["uint8"], 0)

def bus_uint8_set(request):
    registers["uint8"] = request.value
    return BusUint8SetResponse(0)

def main():
    rospy.init_node("bus_server_server")
    rospy.Service("bus_bool_get", BusBoolGet, bus_bool_get)
    rospy.Service("bus_bool_set", BusBoolSet, bus_bool_set)
    rospy.Service("bus_int16_get", BusInt16Get, bus_int16_get)
    rospy.Service("bus_int16_set", BusInt16Set, bus_int16_set)
    rospy.Service("bus_int32_get", BusInt32Get, bus_int32_get)
    rospy.Service("bus_int32_set", BusInt32Set, bus_int32_set)
    rospy.Service("bus_int64_get", BusInt64Get, bus_int64_get)
    rospy.Service("bus_int64_set", BusInt64Set, bus_int64_set)
    rospy.Service("bus_int8_get", BusInt8Get, bus_int8_get)
    rospy.Service("bus_int8_set", BusInt8Set, bus_int8_set)
    rospy.Service("bus_uint16_get", BusUInt16Get, bus_uint16_get)
    rospy.Service("bus_uint16_set", BusUInt16Set, bus_uint16_set)
    rospy.Service("bus_uint32_get", BusUInt32Get, bus_uint32_get)
    rospy.Service("bus_uint32_set", BusUInt32Set, bus_uint32_set)
    rospy.Service("bus_uint64_get", BusUInt64Get, bus_uint64_get)
    rospy.Service("bus_uint64_set", BusUInt64Set, bus_uint64_set)
    rospy.Service("bus_uint8_get", BusUInt8Get, bus_uint8_get)
    rospy.Service("bus_uint8_set", BusUInt8Set, bus_uint8_set)
    print("Starting bus_server_server")
    rospy.spin()

if __name__ == "__main__":
    main()
