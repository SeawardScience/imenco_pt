#pragma once


#include "package_defs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <imenco_pt_interfaces/msg/raw_packet.h>

#include "packets/pf.hpp"
#include "packets/gl.hpp"
#include "packets/set_stop.hpp"
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

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
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
    int home_btn = 0;
    struct{
      float pan_deg = 180;
      float tilt_deg = 180;
    } home_pos;
    struct
    {
      int ccw  = 1;
      int cw   = 2;
      int up   = 3;
      int down = 4;
    } limit_btn;
    int ignore_limit_btn = 5;
    std::string frame_id = "pan_tilt";
  }params_;
  struct{
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  }pubs_;
  struct{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy;
  }subs_;
  rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
  std::shared_ptr<UdpSocket> sock_ptr_;
  packets::PFCmd pf_cmd_;
  packets::PFResp pf_resp_;
  packets::GLCmd gl_cmd_;
  packets::GLResp gl_resp_;
  packets::ESCmd es_cmd_;
  rclcpp::Time last_joy_time_;
  int stop_counter = 0;
  bool time_warn_;
  bool return_to_home_ = false;

};

NS_FOOT
