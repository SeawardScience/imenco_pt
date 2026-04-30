#pragma once


#include "package_defs.hpp"

#include <deque>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <imenco_pt_interfaces/msg/raw_packet.h>

#include "packets/pf.hpp"
#include "packets/gl.hpp"
#include "packets/set_stop.hpp"
#include "packets/ed.hpp"
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
  void producePositionDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void produceEndstopDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void produceErrorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void produceCommsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter>& parameters);
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
    struct
    {
      int ccw  = 1;
      int cw   = 2;
      int up   = 3;
      int down = 4;
    } limit_btn;
    int ignore_limit_btn = 5;
    std::string frame_id = "pan_tilt";
    int checksum_warn_threshold = 5;
    int pan_speed  = 100;
    int tilt_speed = 100;
    float joy_deadband = 0.2f;
    int minimum_speed  = 50;
    std::string hardware_id = "Imenco PT";
  }params_;
  struct{
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
  }pubs_;
  struct{
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy;
  }subs_;
  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_; ///< Shared pointer to the timer
  std::shared_ptr<UdpSocket> sock_ptr_;
  packets::PFCmd pf_cmd_;
  packets::PFResp pf_resp_;
  packets::GLCmd gl_cmd_;
  packets::GLResp gl_resp_;
  packets::ESCmd es_cmd_;
  packets::DSCmd ds_cmd_;
  packets::TACmd ta_cmd_;
  packets::EDCmd ed_cmd_;
  packets::EDResp ed_resp_;
  rclcpp::Time last_joy_time_;
  rclcpp::Time last_response_time_;
  int stop_counter = 0;
  int diag_counter_ = 0;
  uint8_t last_error_byte_ = 0;
  int checksum_error_count_ = 0;
  std::deque<rclcpp::Time> checksum_error_times_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  bool time_warn_;
  bool return_to_home_ = false;

};

NS_FOOT
