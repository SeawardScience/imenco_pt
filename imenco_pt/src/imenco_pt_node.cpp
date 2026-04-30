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

  this->declare_parameter("checksum_warn_threshold", params_.checksum_warn_threshold);
  this->get_parameter("checksum_warn_threshold", params_.checksum_warn_threshold);

  this->declare_parameter("pan_speed", params_.pan_speed);
  this->get_parameter("pan_speed", params_.pan_speed);

  this->declare_parameter("tilt_speed", params_.tilt_speed);
  this->get_parameter("tilt_speed", params_.tilt_speed);

  this->declare_parameter("hardware_id", params_.hardware_id);
  this->get_parameter("hardware_id", params_.hardware_id);

  this->declare_parameter("joy_deadband", params_.joy_deadband);
  this->get_parameter("joy_deadband", params_.joy_deadband);

  this->declare_parameter("minimum_speed", params_.minimum_speed);
  this->get_parameter("minimum_speed", params_.minimum_speed);

  sock_ptr_.reset(new UdpSocket(params_.port));

  pf_cmd_.initalize(params_.to_addr, params_.from_addr);
  gl_cmd_.initalize(params_.to_addr, params_.from_addr);
  es_cmd_.initalize(params_.to_addr, params_.from_addr);
  ed_cmd_.initalize(params_.to_addr, params_.from_addr);
  ds_cmd_.initalize(params_.to_addr, params_.from_addr);
  ds_cmd_.setSpeed(params_.pan_speed);
  ta_cmd_.initalize(params_.to_addr, params_.from_addr);
  ta_cmd_.setSpeed(params_.tilt_speed);
  pubs_.joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  updater_.setHardwareID(params_.hardware_id);
  updater_.add("Position", this, &ImencoPtNode::producePositionDiagnostics);
  updater_.add("Endstops", this, &ImencoPtNode::produceEndstopDiagnostics);
  updater_.add("Errors",   this, &ImencoPtNode::produceErrorDiagnostics);
  updater_.add("Comms", this, &ImencoPtNode::produceCommsDiagnostics);


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

  sock_ptr_->SendTo(params_.dst_ip, params_.port, ds_cmd_.serialize());
  sock_ptr_->SendTo(params_.dst_ip, params_.port, ta_cmd_.serialize());

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ImencoPtNode::onParameterChange, this, std::placeholders::_1));

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

  float pan_raw  = msg->axes[params_.pan_axis];
  float tilt_raw = msg->axes[params_.tilt_axis];

  // Joystick deadband
  if (std::abs(pan_raw)  < params_.joy_deadband) pan_raw  = 0.0f;
  if (std::abs(tilt_raw) < params_.joy_deadband) tilt_raw = 0.0f;

  // Single-axis priority: when both axes active, command only the dominant one
  if (pan_raw != 0.0f && tilt_raw != 0.0f) {
    if (std::abs(pan_raw) >= std::abs(tilt_raw)) tilt_raw = 0.0f;
    else                                          pan_raw  = 0.0f;
  }

  // Apply gain
  float pan_cmd  = pan_raw  * params_.pan_gain;
  float tilt_cmd = tilt_raw * params_.tilt_gain;

  // Minimum speed: snap non-zero commands up to minimum
  float min_frac = params_.minimum_speed / 100.0f;
  if (pan_cmd  != 0.0f && std::abs(pan_cmd)  < min_frac)
    pan_cmd  = std::copysign(min_frac, pan_cmd);
  if (tilt_cmd != 0.0f && std::abs(tilt_cmd) < min_frac)
    tilt_cmd = std::copysign(min_frac, tilt_cmd);

  pf_cmd_.setPan(pan_cmd);
  pf_cmd_.setTilt(tilt_cmd);

  if(pan_raw == 0.0f && tilt_raw == 0.0f && msg->buttons[params_.home_btn]){
    return_to_home_ = true;
  }
  if(pan_raw != 0.0f || tilt_raw != 0.0f){
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
  auto record_checksum_error = [this](){
    checksum_error_count_++;
    checksum_error_times_.push_back(this->now());
  };

  bool cksum_ok = false;
  if(datagram.size() >= pf_resp_.size() && pf_resp_.deserialize(datagram)){
    if(pf_resp_.verifyChecksum()){ pf_resp_.getPos(pan,tilt); cksum_ok = true; }
    else { record_checksum_error(); }
  }
  if(datagram.size() >= gl_resp_.size() && gl_resp_.deserialize(datagram)){
    if(gl_resp_.verifyChecksum()){ gl_resp_.getPos(pan,tilt); cksum_ok = true; }
    else { record_checksum_error(); }
  }
  if(datagram.size() >= ed_resp_.size() && ed_resp_.deserialize(datagram)){
    if(ed_resp_.verifyChecksum()){ last_error_byte_ = ed_resp_.data.error_byte; cksum_ok = true; }
    else { record_checksum_error(); }
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
  stat.add("pan_position_deg",       pan);
  stat.add("tilt_position_deg",      tilt);
  stat.add("pan_speed_device",   (int)pf_resp_.data.pan_speed);
  stat.add("tilt_speed_device",  (int)pf_resp_.data.tilt_speed);
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
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "No response from device");
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

rcl_interfaces::msg::SetParametersResult ImencoPtNode::onParameterChange(
  const std::vector<rclcpp::Parameter>& parameters)
{
  for (const auto& p : parameters) {
    if (p.get_name() == "pan_speed") {
      params_.pan_speed = p.as_int();
      ds_cmd_.setSpeed(params_.pan_speed);
      sock_ptr_->SendTo(params_.dst_ip, params_.port, ds_cmd_.serialize());
      RCLCPP_INFO(this->get_logger(), "Pan speed set to %d", params_.pan_speed);
    } else if (p.get_name() == "tilt_speed") {
      params_.tilt_speed = p.as_int();
      ta_cmd_.setSpeed(params_.tilt_speed);
      sock_ptr_->SendTo(params_.dst_ip, params_.port, ta_cmd_.serialize());
      RCLCPP_INFO(this->get_logger(), "Tilt speed set to %d", params_.tilt_speed);
    } else if (p.get_name() == "joy_deadband") {
      params_.joy_deadband = static_cast<float>(p.as_double());
      RCLCPP_INFO(this->get_logger(), "Joy deadband set to %f", params_.joy_deadband);
    } else if (p.get_name() == "minimum_speed") {
      params_.minimum_speed = p.as_int();
      RCLCPP_INFO(this->get_logger(), "Minimum speed set to %d", params_.minimum_speed);
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void ImencoPtNode::produceCommsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  rclcpp::Time now = this->now();
  while(!checksum_error_times_.empty() &&
        (now - checksum_error_times_.front()).seconds() > 60.0){
    checksum_error_times_.pop_front();
  }
  int recent = static_cast<int>(checksum_error_times_.size());

  rclcpp::Duration age = now - last_response_time_;
  if (age.seconds() > 2.0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No response from device");
  } else if (recent >= params_.checksum_warn_threshold) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
      std::to_string(recent) + " checksum errors in last 60s");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
  stat.add("errors_last_60s", recent);
  stat.add("total_errors",    checksum_error_count_);
  stat.add("warn_threshold",  params_.checksum_warn_threshold);
}

NS_FOOT
