#pragma once

#include "package_defs.h"

NS_HEAD

template <typename data_T>
struct Packet{
  char head[10];
  data_T data;
  char tail[5];

  byte to_ptr(){
    return reinterpret_cast<byte>(head+1);
  }
  byte from_ptr(){
    return reinterpret_cast<byte>(head+3);
  }
  byte length_ptr(){
    return reinterpret_cast<byte>(head+5);
  }
  byte cmd_ptr(){
    return reinterpret_cast<byte>(head+7);
  }
}__attribute__((packed));

NS_FOOT
