cmake_minimum_required(VERSION 3.8)
project(science_microscope)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# further dependencies can be added above as needed

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Build the executable
add_executable(science_microscope_node src/science_microscope.cpp)

target_include_directories(science_microscope_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(
  science_microscope_node
  rclcpp
  sensor_msgs
)

target_link_libraries(science_microscope_node ${OpenCV_LIBS})

install(TARGETS
  science_microscope_node
  DESTINATION lib/${PROJECT_NAME})

## No include directory to install
# install(DIRECTORY include/
#   DESTINATION include)
ament_package()