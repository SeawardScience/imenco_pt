#pragma once

#include <stdint.h>

///
///  Namespace stuff
///
/*#define NS_HEAD  \
  namespace CMAKE_PACKAGE_NAME { */

#define NS_HEAD  \
  namespace imenco_pt {

#define NS_FOOT  \
  }

NS_HEAD
using byte = uint8_t;
NS_FOOT

