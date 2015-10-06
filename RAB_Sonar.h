// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef RAB_SONAR_H_INCLUDED
#define RAB_SONAR_H_INCLUDED 1

#include <Bus_Slave.h>
#include <Sonar.h>

// This class is an abstract base class that defines the
// interfaces needed by the ROS Arduino bridge to access
// the sonars.  It *must* be sub-classed and at a minimum
// a *ping_get*() must be defined.

class RAB_Sonar {
 public:
  RAB_Sonar(UART *debug_uart);
  virtual UShort ping_get(UByte sonar) = 0;
  virtual void configure(UByte sonar_index,
   Sonar_Class sonar_class, Byte left_id, Byte right_id) = 0;
  virtual UShort debug_flags_get() = 0;
  virtual void debug_flags_set(UShort debug_flags) = 0;
  virtual void queue_poll(UART *host_uart,
   UInteger time_base, UByte id_offset) = 0;
  virtual void direction_set(Byte direction) = 0;  
  virtual UByte sonars_count_get() = 0;
 protected:
  UART *debug_uart_;
};

#endif // RAB_SONAR_H_INCLUDED
