cmake_minimum_required(VERSION 3.8)
project(openarm_bilateral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# added
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(bilateral_openarm_main
  src/canbus.cpp
  src/damiao_port.cpp
  src/motor_control.cpp
  src/motor.cpp
  src/dynamics.cpp
  src/control.cpp
  src/bilateral_main.cpp
  #src/test01.cpp
)

ament_target_dependencies(bilateral_openarm_main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  urdf
  kdl_parser
  orocos_kdl
)

target_include_directories(bilateral_openarm_main PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}
)

install(TARGETS bilateral_openarm_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
