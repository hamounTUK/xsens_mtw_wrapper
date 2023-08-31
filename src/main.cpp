// ROS dependencies
// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"

#include <string>

// Internal dependencies
#include "xsens_mtw/Wrapper.h"

int main(int argc, char* argv[])
{
  std::string node_name = "hiros_xsens_mtw_wrapper";
  // XsDevicePtr m_wireless_master_device;
  // typedef std::set<XsDevice*> XsDeviceSet;


  std::cout << "NODE: " << node_name << std::endl;
  // ros::init(argc, argv, node_name);

  // hiros::xsens_mtw::Wrapper wrapper;
  // wrapper.start();
  // wrapper.run();



  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hiros::xsens_mtw::Wrapper>());
  
  rclcpp::shutdown();

  return 0;
}
