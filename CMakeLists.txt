cmake_minimum_required(VERSION 3.8)
project(i2c_service)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(i2c_interfaces REQUIRED)

set(dependencies "rclcpp" "i2c_interfaces")

include_directories(include)

# Add executable and link dependencies
add_executable(i2c_service src/i2c_service.cpp src/i2c_port.cpp)
ament_target_dependencies(i2c_service ${dependencies})
target_link_libraries(i2c_service "i2c")

# Install executables in lib
install(
	TARGETS i2c_service 
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
