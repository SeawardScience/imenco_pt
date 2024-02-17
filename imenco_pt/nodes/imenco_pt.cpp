#include "imenco_pt_node.hpp"
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  imenco_pt::packets::PFCmd cmd;
  auto node =  std::make_shared<imenco_pt::ImencoPtNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;

}
