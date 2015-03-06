// Copyright (c) 2014 by Wayne C. Gramlich.  All rights reserved.

#include "Frame_Buffer.h"

Frame_Buffer::Frame_Buffer() {
  _in_index = 0;
  _out_index = 0;
}

void Frame_Buffer::append(UShort frame) {
  _buffer[_in_index++ & _buffer_mask] = frame;
}

Logical Frame_Buffer::is_empty() {
  return (Logical)(_in_index == _out_index);
}

Logical Frame_Buffer::is_full() {
  return (Logical)(((_in_index + 1) & _buffer_mask) == _out_index);
}

UShort Frame_Buffer::lop() {
  return _buffer[_out_index++ & _buffer_mask];
}
