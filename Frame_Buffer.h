// Copyright (c) 2014 by Wayne C. Gramlich.  All rights reserved.

#ifndef BUS_FRAME_BUFFER_H
#define BUS_FRAME_BUFFER_H 1

#include "Bus_Slave.h"

class Frame_Buffer {
  public:
    Frame_Buffer();		// Constructor
    void append(UShort frame);	// Append *frame* to buffer
    Logical is_empty();		// 1 => buffer is empty
    Logical is_full();		// 1 => buffer is full
    UShort lop();		// Remove *frame* from buffer

  private:
    static const UByte _buffer_power = 4;// Size as a power of 2
    static const UByte _buffer_size = 1 << _buffer_power; // Actual size
    static const UByte _buffer_mask = _buffer_size - 1;   // Index mask
    UShort _buffer[_buffer_size];	// Actual buffer
    UByte _in_index;			// Input index
    UByte _out_index;			// Output index
};

#endif // BUS_FRAME_BUFFER_H
