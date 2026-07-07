#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(viper)
set(VIPER_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#######################################################################################
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
##########################################################################
add_subdirectory(external/107-Arduino-Cyphal)
add_subdirectory(external/libsocketcan)
#######################################################################################
include_directories(include)
#######################################################################################
# Silence the GSL compatibility mode warnings
add_definitions(-Dgsl_FEATURE_GSL_COMPATIBILITY_MODE=1)

add_executable(${VIPER_TARGET}
  src/CanManager.cpp
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_link_libraries(${VIPER_TARGET} cyphal++ socketcan)
#######################################################################################
target_compile_features(${VIPER_TARGET} PRIVATE cxx_std_20)
ament_target_dependencies(${VIPER_TARGET} rclcpp std_msgs sensor_msgs geometry_msgs actuator_msgs)
#######################################################################################
install(TARGETS ${VIPER_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY gazebo DESTINATION share/${PROJECT_NAME})
#######################################################################################
# Add environment variables for Gazebo
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/gazebo/env-hooks/${PROJECT_NAME}.dsv.in")
#######################################################################################
ament_package()
#######################################################################################
