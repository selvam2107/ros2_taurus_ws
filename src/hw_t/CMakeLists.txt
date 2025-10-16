cmake_minimum_required(VERSION 3.8)
project(hw_t)

# ================================
# Compiler settings
# ================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================
# Dependencies
# ================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Optional: system libmodbus
find_package(PkgConfig)
pkg_check_modules(MODBUS libmodbus)

# ================================
# Include directories
# ================================
include_directories(
  include
  ${MODBUS_INCLUDE_DIRS}
)

# ================================
# Shared library for Moons class
# ================================
add_library(moons_lib
  src/moonsModbus.cpp
)

ament_target_dependencies(moons_lib
  rclcpp
  std_msgs
)

# Link libmodbus if found
if(MODBUS_FOUND)
  target_link_libraries(moons_lib ${MODBUS_LIBRARIES})
endif()

# ================================
# Executable: hardware_interface_node
# ================================
add_executable(hardware_interface_node
  src/hardware_interface.cpp
)

ament_target_dependencies(hardware_interface_node
  rclcpp
  std_msgs
)

target_link_libraries(hardware_interface_node
  moons_lib
)

# ================================
# Executable: hardware_test_node
# ================================
add_executable(hardware_test_node
  src/test_hardware_interface.cpp
)

ament_target_dependencies(hardware_test_node
  rclcpp
  std_msgs
)

target_link_libraries(hardware_test_node
  moons_lib
)

# ================================
# Installation
# ================================
install(TARGETS
  hardware_interface_node
  hardware_test_node
  moons_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ================================
# Package export
# ================================
ament_package()
