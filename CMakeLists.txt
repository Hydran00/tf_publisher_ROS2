cmake_minimum_required(VERSION 3.8)
project(tf_publisher)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  tf2_ros
  geometry_msgs
  std_msgs
)
#C++

add_executable(tf_publisher src/tf_publisher.cpp)

ament_target_dependencies(tf_publisher ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS
tf_publisher
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()


