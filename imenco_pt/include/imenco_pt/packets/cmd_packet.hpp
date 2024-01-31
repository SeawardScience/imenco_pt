#pragma once

#include "packets_defs.hpp"

#include <stddef.h>
#include <vector>

PACKETS_NS_HEAD

#define HEAD_SIZE 10
#define TAIL_SIZE 5



struct None{
};

template <typename data_T>
struct CmdPacket{
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
  char* cmd_id_ptr(){return head_as<char>(7);}

  /*!
   * \brief initalize adds the separator characters and the < > to the head and the tail
   * of the message.
   * \note This mehod will overide any data in the message
   */
  void initalize(uint8_t to = 3, uint8_t from = 1){
    auto head = head_as<char>();
    head[0] = '<';
    for(size_t i = 1; i < HEAD_SIZE; i++){
      head[i]=':';
    }

    auto tail = tail_as<char>();
    for(size_t i = 0; i < TAIL_SIZE; i++){
      tail[i]=':';
    }
    tail[TAIL_SIZE-1]='>';

    cmd_id_ptr()[0] = data.id()[0];
    cmd_id_ptr()[1] = data.id()[1];

    *to_ptr() = to;
    *from_ptr() = from;
    *length_ptr() =  sizeof(data_T) + 3;
  }

  void computeChecksum(){
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
    *tail_as<uint8_t>(1) =  cksum;
  }

  void setBit(uint8_t * data, uint8_t index, bool state){
    if(state)
      *data |= (1 << index);
    else
      *data &= ~(1 << index);
  }

  std::vector<byte> serialize(){
    computeChecksum();
    std::vector<byte> out;
    out.resize(size());
    for(size_t i=0; i<size() ; i++){
      out[i] = *this->as<byte>(i);
    }
    return out;
  }

  size_t size(){return sizeof(CmdPacket<data_T>);}

}__attribute__((packed));

PACKETS_NS_FOOT
