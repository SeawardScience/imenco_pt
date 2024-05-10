#pragma once

#include "cmd_packet.hpp"
#include "resp_packet.hpp"

#include <stdlib.h>     /* abs */
#include <string>
#include <algorithm>
#include <iostream>>

PACKETS_NS_HEAD

struct AWCmdData{
  char * id(){return "AW";}
};
class AWCmd : public CmdPacket<AWCmdData> {};

struct CWCmdData{
  char * id(){return "CW";}
};
class CWCmd : public CmdPacket<CWCmdData> {};

struct UTCmdData{
  char * id(){return "UT";}
};
class UTCmd : public CmdPacket<UTCmdData> {};

struct DTCmdData{
  char * id(){return "DT";}
};
class DTCmd : public CmdPacket<DTCmdData> {};

struct ESCmdData{
  byte use_stops;
  char * id(){return "ES";}
};
class ESCmd : public CmdPacket<ESCmdData> {
public:
  void useStops(bool use_stops){
    if(use_stops){
      data.use_stops = 48;
    }else{
      data.use_stops = 49;
    }
  }
};

PACKETS_NS_FOOT
