#include "imenco_pt_node.hpp"

using namespace std::chrono_literals;

NS_HEAD

ImencoPtNode::ImencoPtNode()
  : Node("imenco_pt")
{
  params_.dst_ip = "10.0.0.31";
  this->declare_parameter("dst_ip", params_.dst_ip);
  this->get_parameter("dst_ip",params_.dst_ip);

  params_.port = 4016;
  this->declare_parameter("port", params_.port);
  this->get_parameter("port",params_.port);

  params_.to_addr = 3;
  this->declare_parameter("to_addr", params_.to_addr);
  this->get_parameter("to_addr",params_.to_addr);

  params_.from_addr = 1;
  this->declare_parameter("from_addr", params_.from_addr);
  this->get_parameter("from_addr",params_.from_addr);

  params_.joy_topic = "joy_console/joy";
  this->declare_parameter("joy_topic", params_.joy_topic);
  this->get_parameter("joy_topic",params_.joy_topic);

  params_.pan_axis = 1;
  this->declare_parameter("pan_axis", params_.pan_axis);
  this->get_parameter("pan_axis",params_.pan_axis);

  params_.tilt_axis = 2;
  this->declare_parameter("tilt_axis", params_.tilt_axis);
  this->get_parameter("tilt_axis",params_.tilt_axis);

  params_.pan_gain = -0.1;
  this->declare_parameter("pan_gain", params_.pan_gain);
  this->get_parameter("pan_gain",params_.pan_gain);

  params_.tilt_gain = 0.1;
  this->declare_parameter("tilt_gain", params_.tilt_gain);
  this->get_parameter("tilt_gain",params_.tilt_gain);

  params_.max_joy_age = 0.5;
  this->declare_parameter("max_joy_age", params_.max_joy_age);
  this->get_parameter("max_joy_age",params_.max_joy_age);

  sock_ptr_.reset(new UdpSocket(params_.port));

  pf_cmd_.initalize(params_.to_addr, params_.from_addr);

  subs_.joy = this->create_subscription<sensor_msgs::msg::Joy>(
        params_.joy_topic, 1, std::bind(&ImencoPtNode::joyCallback, this, std::placeholders::_1));

  last_joy_time_ = this->now();

  time_warn_ = false;

  timer_ = this->create_wall_timer(
    100ms, std::bind(&ImencoPtNode::timer_callback, this));

  sock_ptr_->AddCallback(std::bind(&ImencoPtNode::udpCallback,
                              this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Waiting for joy message on topic: %s", subs_.joy->get_topic_name());
  RCLCPP_INFO(this->get_logger(), "Sending messages to IP: %s, Port: %i", params_.dst_ip.c_str(),params_.port);
}

void ImencoPtNode::timer_callback()
{
  rclcpp::Duration age = this->now() - last_joy_time_;
  if(age.seconds()>params_.max_joy_age){
    if(time_warn_)
      RCLCPP_WARN(this->get_logger(), "No Joy message received in the last %f sec.  Zeroing Commands",params_.max_joy_age);
    pf_cmd_.setPan(0);
    pf_cmd_.setTilt(0);
    time_warn_ = false;
  }
  sock_ptr_->SendTo(params_.dst_ip, params_.port,pf_cmd_.serialize());
  sock_ptr_->Receive();
}

void ImencoPtNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{


  last_joy_time_ = msg->header.stamp;
  pf_cmd_.setPan(msg->axes[params_.pan_axis]*params_.pan_gain);
  pf_cmd_.setTilt(msg->axes[params_.tilt_axis]*params_.tilt_gain);

  rclcpp::Duration age = this->now() - last_joy_time_;

  if(age.seconds()<params_.max_joy_age){
    if(!time_warn_){
      RCLCPP_INFO(this->get_logger(), "Valid joy message received");
    }
    time_warn_ = true;
  }

}

void ImencoPtNode::udpCallback(const std::vector<byte> &datagram)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Received Response From PT unit");
  int pan, tilt;
  pf_resp_.deserialize(datagram);
  pf_resp_.getPanPos(pan,tilt);

  //RCLCPP_INFO(this->get_logger(), "%i,%i",pan,tilt);

  //TODO:   pan and tilt are parsing properly.   We just need to do something with them

  return;
}

NS_FOOT
