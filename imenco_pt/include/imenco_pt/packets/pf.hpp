#pragma once

#include "cmd_packet.hpp"

#include <stdlib.h>     /* abs */
#include <string>

PACKETS_NS_HEAD

struct PFCmdData{
  byte cmd_action = 0;
  byte pan_speed = 0;
  byte tilt_speed = 0;
  byte reserved = 0;

  char * id(){return "PF";}
};

class PFCmd : public CmdPacket<PFCmdData> {
public:
  static const uint8_t MAX_SPEED = 0x64;
  /*!
   * \brief setPan sets the pan speed and direction
   * \param value a value between -1.0 and 1.0.  Negative values pan left
   * positive values pan right.
   */
  void setPan(float value){
    if(value>0){
      setBit(&data.cmd_action,0,0);
      setBit(&data.cmd_action,1,1);
    }
    if(value<0){
      setBit(&data.cmd_action,0,1);
      setBit(&data.cmd_action,1,0);
    }
    if(value==0){
      setBit(&data.cmd_action,0,0);
      setBit(&data.cmd_action,1,0);
    }
    data.pan_speed=abs(value)*MAX_SPEED;
  }
  void setTilt(float value){
    if(value>0){
      setBit(&data.cmd_action,2,1);
      setBit(&data.cmd_action,3,0);
    }
    if(value<0){
      setBit(&data.cmd_action,2,0);
      setBit(&data.cmd_action,3,1);
    }
    if(value==0){
      setBit(&data.cmd_action,2,0);
      setBit(&data.cmd_action,3,0);
    }
    data.tilt_speed=abs(value)*MAX_SPEED;
  }
};

PACKETS_NS_FOOT
