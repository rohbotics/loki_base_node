// Copyright (c) 2015 by Wayne C. Gramlich.  All rights reserved.

#include <Bus_Slave.h>
#include <RAB_Sonar.h>

RAB_Sonar::RAB_Sonar(UART *debug_uart) {
  debug_uart_ = debug_uart;
}
