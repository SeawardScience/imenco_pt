#pragma once

#include "packets_defs.hpp"

#include <stddef.h>
#include <vector>

PACKETS_NS_HEAD

template <typename data_T>
struct RespPacket{

  static const size_t HEAD_SIZE = 11;
  static const size_t TAIL_SIZE = 5;

  char head[HEAD_SIZE];
  data_T data;
  char tail[TAIL_SIZE];

  template <typename output_T>
  output_T* head_as(size_t i = 0){
    return reinterpret_cast<output_T*>(head+i);
  }

  template <typename output_T>
  output_T* data_as(size_t i = 0){
    return reinterpret_cast<output_T*>(&data)+i;
  }

  template <typename output_T>
  output_T* tail_as(size_t i = 0){
    return reinterpret_cast<output_T*>(tail+i);
  }

  template <typename output_T>
  output_T* as(size_t i = 0 ){
    return reinterpret_cast<output_T*>(this)+i;
  }

  byte* to_ptr(){return head_as<byte>(1);}
  byte* from_ptr(){return head_as<byte>(3);}
  byte* length_ptr(){return head_as<byte>(5);}
  char* ack_ptr(){return head_as<char>(7);}
  char* cmd_id_ptr(){return head_as<char>(9);}
  byte* cksum_ptr(){tail_as<uint8_t>(1);}

  byte computeChecksum(){
    byte cksum = 0;
    for(size_t i = 1; i<HEAD_SIZE; i++){
      auto val = *head_as<uint8_t>(i);
      cksum ^= *head_as<uint8_t>(i);
    }
    for(size_t i = 0; i<sizeof(data_T); i++){
      auto val = *data_as<uint8_t>(i);
      cksum ^= *data_as<uint8_t>(i);
    }
    if(cksum == 0x3C){
      *tail_as<char>(3) = '0';
      cksum = 0xFF;
    }else if(cksum == 0x3E){
      *tail_as<char>(3) = '1';
      cksum = 0xFF;
    }else{
      *tail_as<char>(3) = 'G';
    }
    //*tail_as<uint8_t>(1) =  cksum;
    return cksum;
  }

  void deserialize(const std::vector<byte> &datagram){
    for(size_t i = 0 ; i < size() ; i++){
      *head_as<byte>(i) = datagram[i];
    }
  }

  size_t size(){return sizeof(CmdPacket<data_T>);}

}__attribute__((packed));

PACKETS_NS_FOOT
