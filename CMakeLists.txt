cmake_minimum_required(VERSION 3.8)
project(tca9548a)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectChannel.srv"
)

add_library(tca9548a_driver src/tca9548a.cpp)

target_include_directories(tca9548a_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(tca9548a_node src/tca9548a_node.cpp)

ament_target_dependencies(tca9548a_node
  "rclcpp"
)

rosidl_target_interfaces(tca9548a_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(tca9548a_node tca9548a_driver)

install(TARGETS
  tca9548a_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tca9548a_driver
  EXPORT export_tca9548a_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_targets(export_tca9548a_driver
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()