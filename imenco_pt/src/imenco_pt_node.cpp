#include "imenco_pt_node.hpp"

using namespace std::chrono_literals;

NS_HEAD

ImencoPtNode::ImencoPtNode()
  : Node("imenco_pt"), updater_(this)
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

  this->declare_parameter("home_btn", params_.home_btn);
  this->get_parameter("home_btn",params_.home_btn);

  this->declare_parameter("frame_id", params_.frame_id);
  this->get_parameter("frame_id",params_.frame_id);

  sock_ptr_.reset(new UdpSocket(params_.port));

  pf_cmd_.initalize(params_.to_addr, params_.from_addr);
  gl_cmd_.initalize(params_.to_addr, params_.from_addr);
  es_cmd_.initalize(params_.to_addr, params_.from_addr);
  ed_cmd_.initalize(params_.to_addr, params_.from_addr);
  pubs_.joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  updater_.setHardwareID(this->get_name());
  updater_.add("Position", this, &ImencoPtNode::producePositionDiagnostics);
  updater_.add("Endstops", this, &ImencoPtNode::produceEndstopDiagnostics);
  updater_.add("Errors",   this, &ImencoPtNode::produceErrorDiagnostics);


  subs_.joy = this->create_subscription<sensor_msgs::msg::Joy>(
        params_.joy_topic, 1, std::bind(&ImencoPtNode::joyCallback, this, std::placeholders::_1));

  last_joy_time_ = this->now();
  last_response_time_ = this->now();

  time_warn_ = false;

  timer_ = this->create_wall_timer(
    200ms, std::bind(&ImencoPtNode::timer_callback, this));

  sock_ptr_->AddCallback(std::bind(&ImencoPtNode::udpCallback,
                              this, std::placeholders::_1));

  gl_cmd_.setPos(180,180);

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


  // auto cmd = es_cmd_.serialize();
  // for (auto byte : cmd) {
  //   std::cout << static_cast<char>(byte);
  // }
  // std::cout << std::endl; // End line after printing all characters

  if(stop_counter>4){
    sock_ptr_->SendTo(params_.dst_ip, params_.port,es_cmd_.serialize());
    stop_counter = 0;
  }else{
    if(return_to_home_){
      sock_ptr_->SendTo(params_.dst_ip, params_.port,gl_cmd_.serialize());
    }else{
      sock_ptr_->SendTo(params_.dst_ip, params_.port,pf_cmd_.serialize());
    }
    stop_counter++;
  }




  diag_counter_++;
  if (diag_counter_ > 4) {
    sock_ptr_->SendTo(params_.dst_ip, params_.port, ed_cmd_.serialize());
    diag_counter_ = 0;
  }

  while (sock_ptr_->Receive()) {}
}

void ImencoPtNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{


  last_joy_time_ = msg->header.stamp;
  pf_cmd_.setPan(msg->axes[params_.pan_axis]*params_.pan_gain);
  pf_cmd_.setTilt(msg->axes[params_.tilt_axis]*params_.tilt_gain);

  if(msg->axes[params_.pan_axis] == 0.0 && msg->axes[params_.tilt_axis] == 0.0 && msg->buttons[params_.home_btn]){
    return_to_home_ = true;
  }
  if(msg->axes[params_.pan_axis] != 0.0 || msg->axes[params_.tilt_axis] != 0.0){
    return_to_home_ = false;
  }

  int pan, tilt;
  pf_resp_.getPos(pan,tilt);


  if(msg->buttons[params_.limit_btn.ccw]){
    RCLCPP_INFO(this->get_logger(), "Setting AW Limit At: %i", pan);
    packets::AWCmd cmd;
    cmd.initalize(params_.to_addr, params_.from_addr);
    sock_ptr_->SendTo(params_.dst_ip, params_.port,cmd.serialize());
  }
  if(msg->buttons[params_.limit_btn.cw]){
    RCLCPP_INFO(this->get_logger(), "Setting CW Limit At: %i", pan);
    packets::CWCmd cmd;
    cmd.initalize(params_.to_addr, params_.from_addr);
    sock_ptr_->SendTo(params_.dst_ip, params_.port,cmd.serialize());
  }

  if(msg->buttons[params_.limit_btn.up]){
    packets::UTCmd cmd;
    cmd.initalize(params_.to_addr, params_.from_addr);
    sock_ptr_->SendTo(params_.dst_ip, params_.port,cmd.serialize());
  }

  if(msg->buttons[params_.limit_btn.down]){
    packets::DTCmd cmd;
    cmd.initalize(params_.to_addr, params_.from_addr);
    sock_ptr_->SendTo(params_.dst_ip, params_.port,cmd.serialize());
  }

  if(msg->buttons[params_.ignore_limit_btn]){
    es_cmd_.useStops(true);
  }else{
    es_cmd_.useStops(false);
  }


  rclcpp::Duration age = this->now() - last_joy_time_;

  if(age.seconds()<params_.max_joy_age){
    if(!time_warn_){
      RCLCPP_INFO(this->get_logger(), "Valid joy message received");
    }
    time_warn_ = true;
  }

  //RCLCPP_INFO(this->get_logger(), "%f,%f",msg->axes[params_.pan_axis]*params_.pan_gain,msg->axes[params_.tilt_axis]*params_.tilt_gain);

}

void ImencoPtNode::udpCallback(const std::vector<byte> &datagram)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Received Response From PT unit");
  last_response_time_ = this->now();

  int pan, tilt;
  if(datagram.size() >= pf_resp_.size() && pf_resp_.deserialize(datagram)){
    pf_resp_.getPos(pan,tilt);
  }
  if(datagram.size() >= gl_resp_.size() && gl_resp_.deserialize(datagram)){
    gl_resp_.getPos(pan,tilt);
  }
  if(datagram.size() >= ed_resp_.size() && ed_resp_.deserialize(datagram)){
    last_error_byte_ = ed_resp_.data.error_byte;
  }

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.frame_id = params_.frame_id;
  joint_state_msg.header.stamp = this->now();
  joint_state_msg.name.push_back("pan_joint");
  joint_state_msg.position.push_back(pan * M_PI / 180);  // Convert degrees to radians
  joint_state_msg.name.push_back("tilt_joint");
  joint_state_msg.position.push_back(tilt * M_PI / 180); // Convert degrees to radians

  pubs_.joint_state_pub->publish(joint_state_msg);
}

void ImencoPtNode::producePositionDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  rclcpp::Duration age = this->now() - last_response_time_;
  if (age.seconds() > 2.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No data");
    return;
  }
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");

  int pan, tilt;
  pf_resp_.getPos(pan, tilt);
  stat.add("pan_position_deg",  pan);
  stat.add("tilt_position_deg", tilt);
  stat.add("pan_speed",         (int)pf_resp_.data.pan_speed);
  stat.add("tilt_speed",        (int)pf_resp_.data.tilt_speed);
}

void ImencoPtNode::produceEndstopDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  rclcpp::Duration age = this->now() - last_response_time_;
  if (age.seconds() > 2.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No data");
    return;
  }

  bool pan_en  = pf_resp_.data.pan_endstops_enable  == 0x31;
  bool tilt_en = pf_resp_.data.tilt_endstops_enable == 0x31;

  if (!pan_en && !tilt_en) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Pan and Tilt endstops disabled");
  } else if (!pan_en) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Pan endstops disabled");
  } else if (!tilt_en) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Tilt endstops disabled");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }

  stat.add("pan_endstops_enabled",  pan_en  ? "true" : "false");
  stat.add("tilt_endstops_enabled", tilt_en ? "true" : "false");
}

void ImencoPtNode::produceErrorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  rclcpp::Duration age = this->now() - last_response_time_;
  if (age.seconds() > 2.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No response from device");
    return;
  }

  if (last_error_byte_ == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  } else {
    std::vector<std::string> faults;
    if (last_error_byte_ & (1 << 0)) faults.push_back("Over Temperature");
    if (last_error_byte_ & (1 << 1)) faults.push_back("Low Oil Level");
    if (last_error_byte_ & (1 << 2)) faults.push_back("Moisture Ingress");
    if (last_error_byte_ & (1 << 3)) faults.push_back("Over Current");
    if (last_error_byte_ & (1 << 4)) faults.push_back("Tilt Stall");
    if (last_error_byte_ & (1 << 5)) faults.push_back("Pan Stall");
    std::string msg;
    for (size_t i = 0; i < faults.size(); ++i) {
      if (i > 0) msg += ", ";
      msg += faults[i];
    }
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, msg);
  }

  stat.add("over_temperature",  (last_error_byte_ & (1 << 0)) ? "true" : "false");
  stat.add("low_oil_level",     (last_error_byte_ & (1 << 1)) ? "true" : "false");
  stat.add("moisture_ingress",  (last_error_byte_ & (1 << 2)) ? "true" : "false");
  stat.add("over_current",      (last_error_byte_ & (1 << 3)) ? "true" : "false");
  stat.add("tilt_stall",        (last_error_byte_ & (1 << 4)) ? "true" : "false");
  stat.add("pan_stall",         (last_error_byte_ & (1 << 5)) ? "true" : "false");
}

NS_FOOT
