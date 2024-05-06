#pragma once

#include "cmd_packet.hpp"
#include "resp_packet.hpp"

#include <stdlib.h>     /* abs */
#include <string>
#include <algorithm>
#include <iostream>>

PACKETS_NS_HEAD

struct GLCmdData{
  char pan[3];
  char tilt[3];

  char * id(){return "GL";}
};

/*!
 * \brief Corresponds to an imenco PF command which sets porpotional pand and tilt controls
 */
class GLCmd : public CmdPacket<GLCmdData> {
public:
  void setPos(int pan, int tilt){
    // Adjust pan and tilt to be within the 0 to 360 range
    int adjusted_pan = ((pan % 360) + 360) % 360;
    int adjusted_tilt = ((tilt % 360) + 360) % 360;

    // Convert integer pan to a three-character array
    snprintf(data.pan, sizeof(data.pan), "%03d", adjusted_pan);
    data.pan[sizeof(data.pan) - 1] = '0'; // Manually set the last character to '0' to avoid null termination

    // Convert integer tilt to a three-character array
    snprintf(data.tilt, sizeof(data.tilt), "%03d", adjusted_tilt);
    data.tilt[sizeof(data.tilt) - 1] = '0'; // Manually set the last character to '0' to avoid null termination
  }
};

struct GLRespData{
  char position_str[6];

  char * id(){return "GL";}
};

class GLResp : public RespPacket<GLRespData>{
public:
  void getPos(int & pan, int & tilt){
    sscanf(data.position_str, "%3d%3d", &tilt, &pan);
  }
};

PACKETS_NS_FOOT
