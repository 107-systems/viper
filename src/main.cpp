/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/viper/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <viper/Node.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<viper::Node>();

  try
  {
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }
  catch (std::runtime_error const & err)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Exception (std::runtime_error) caught: %s\nTerminating ...", err.what());
    return EXIT_FAILURE;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node->get_name()), "Unhandled exception caught.\nTerminating ...");
    return EXIT_FAILURE;
  }
}
