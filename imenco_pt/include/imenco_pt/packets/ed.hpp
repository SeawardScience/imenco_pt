#pragma once

#include "cmd_packet.hpp"
#include "resp_packet.hpp"

PACKETS_NS_HEAD

struct EDCmdData {
  char * id(){return "ED";}
};
class EDCmd : public CmdPacket<EDCmdData> {};

struct EDRespData {
  // Bit0=Over Temperature, Bit1=Low Oil Level, Bit2=Moisture Ingress,
  // Bit3=Over Current, Bit4=Tilt Stall, Bit5=Pan Stall
  byte error_byte = 0;
  char * id(){return "ED";}
};
class EDResp : public RespPacket<EDRespData> {};

PACKETS_NS_FOOT
