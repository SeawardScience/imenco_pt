#pragma once


#include "package_defs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <imenco_pt_interfaces/msg/raw_packet.h>

#include "packets/pf.hpp"
#include "udp_socket.hpp"

NS_HEAD  // macro for consistantly defining our namespace for the package


class ImencoPtNode : public rclcpp::Node
{
public:
  ImencoPtNode();

protected:
  void timer_callback();
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void udpCallback(const std::vector<byte> &datagram);
  //void rawCallback(const imenco_pt_interfaces::msg::RawPacket::SharedPtr msg);
  struct{
    int port;
    std::string dst_ip;
    int to_addr;
    int from_addr;
    std::string joy_topic;
    int pan_axis;
    int tilt_axis;
    float pan_gain;
    float tilt_gain;
    float max_joy_age;
  }params_;
  struct{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy;
  }subs_;
  rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
  std::shared_ptr<UdpSocket> sock_ptr_;
  packets::PFCmd pf_cmd_;
  packets::PFResp pf_resp_;
  rclcpp::Time last_joy_time_;
  bool time_warn_;

};

NS_FOOT
