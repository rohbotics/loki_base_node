// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#ifndef RAB_SONAR_H_INCLUDED
#define RAB_SONAR_H_INCLUDED 1

#include <Bus_Slave.h>

// This class is an abstract base class that defines the
// interfaces needed by the ROS Arduino bridge to access
// the sonars.  It *must* be sub-classed and at a minimum
// a *ping_get*() must be defined.

class RAB_Sonar {
 public:
  virtual Short ping_get(UByte sonar) = 0;
  virtual Short system_debug_flags_get() = 0;
  virtual void system_debug_flags_set(Short system_flags) = 0;
  virtual UByte sonars_count_get() = 0;
};

#endif // RAB_SONAR_H_INCLUDED
