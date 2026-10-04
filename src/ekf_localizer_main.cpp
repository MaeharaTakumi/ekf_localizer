#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ekf_localizer/ekf_localizer_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ekf_localizer::EkfLocalizer>());
  rclcpp::shutdown();
  return 0;
}
