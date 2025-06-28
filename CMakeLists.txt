cmake_minimum_required(VERSION 3.8)
project(tca9548a)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate C++ code from the .srv file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectChannel.srv"
)

# ===================================================================
# == 1. CREATE AND CONFIGURE THE REUSABLE DRIVER LIBRARY           ==
# ===================================================================
# Build the C++ library for the driver from its source file
add_library(tca9548a_driver src/tca9548a.cpp)

# Associate include directories with the library target directly.
# This is the modern replacement for the global include_directories().
# PUBLIC propagates this include path to any other target that links against this library.
target_include_directories(tca9548a_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# ===================================================================
# == 2. DEFINE THE EXECUTABLE NODE                                 ==
# ===================================================================
# Build the ROS2 node executable from its source file
add_executable(tca_manager_node src/tca_manager_node.cpp)

# Specify ROS dependencies for the executable.
ament_target_dependencies(tca_manager_node
  "rclcpp"
)

# This links the node to the generated C++ service code.
rosidl_target_interfaces(tca_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Link the executable to the driver library we just built.
target_link_libraries(tca_manager_node tca9548a_driver)


# ===================================================================
# == 3. SET UP INSTALLATION AND EXPORT RULES                       ==
# ===================================================================
# This section ensures all necessary files are placed in the `install`
# directory and that other packages can find our library.

# Install the executable target.
install(TARGETS
  tca_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the library target and define an export name.
install(TARGETS tca9548a_driver
  EXPORT export_tca9548a_driver # This name is used by ament_export_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the public header files.
install(DIRECTORY
  include/
  DESTINATION include
)

# Export the library target. This is crucial for other packages to be able
# to find and link against `tca9548a_driver` using find_package().
ament_export_targets(export_tca9548a_driver
  HAS_LIBRARY_TARGET
)

# Export the generated service definitions.
ament_export_dependencies(rosidl_default_runtime)

# This command must be at the end of the file.
ament_package()