cmake_minimum_required(VERSION 3.8)
project(tca9548a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pluginlib REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RegisterDevice.srv"
  "srv/InitDevice.srv"
  "srv/ConfigDevice.srv"
  "srv/ReadDevice.srv"
)

add_library(tca9548a_driver src/tca9548a.cpp)
target_include_directories(tca9548a_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(tca9548a_manager src/tca9548a_manager.cpp)
ament_target_dependencies(tca9548a_manager "rclcpp" "pluginlib")
rosidl_get_typesupport_target(tca_typesupport_libs ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tca9548a_manager
  tca9548a_driver
  ${tca_typesupport_libs}
)

# --- Modern Installation & Export Rules for Jazzy ---
install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}/srv
)

install(TARGETS
  tca9548a_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library and CREATE an export set named "tca9548a_driver_targets"
install(TARGETS tca9548a_driver
  EXPORT tca9548a_driver_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Explicitly install the include directory
install(DIRECTORY include/
  DESTINATION include
)

# EXPORT the targets for other packages to find
ament_export_targets(tca9548a_driver_targets
  HAS_LIBRARY_TARGET
)

# Export dependencies needed by consumers of this package
ament_export_dependencies(rosidl_default_runtime)

# Install launch and config
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()