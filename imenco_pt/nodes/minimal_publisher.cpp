#include "minimal_publisher_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imenco_pt::MinimalPublisherNode>());
  rclcpp::shutdown();
  return 0;

}
