cmake_minimum_required(VERSION 3.8)
project(tca9548a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This exposes the "include" directory to be found by C++
include_directories(include)

# Generate C++ code from the .srv file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectChannel.srv"
)

# Build the C++ library for the driver from its source file
add_library(tca9548a_driver src/tca9548a.cpp)

# Build the ROS2 node executable from its source file
add_executable(tca_manager_node src/tca_manager_node.cpp)

# Link the node to its dependencies
ament_target_dependencies(tca_manager_node rclcpp)

# Link the executable to the driver library we just built
target_link_libraries(tca_manager_node tca9548a_driver)

# This is important! It links the node to the generated service code.
rosidl_target_interfaces(tca_manager_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install all the necessary components
install(TARGETS
  tca_manager_node
  tca9548a_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install the public header file
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()