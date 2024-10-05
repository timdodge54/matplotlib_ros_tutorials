cmake_minimum_required(VERSION 3.5)
project(test_plot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++14)

# add_compile_options(-g)
add_compile_options(-march=native -O3 -DNBUG -omit-frame-pointer)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_interfaces REQUIRED)
find_package(test_msg REQUIRED)

# find_package(Boost REQUIRED COMPONENTS system)
# include_directories(
# include
# ${YAML_CPP_INCLUDE_DIRS}
# )

# install(TARGETS
# threshold_operator_planner
# DESTINATION lib/${PROJECT_NAME})

# ament_python_install_package(${PROJECT_NAME})
# ament_python_install_package(UTA_Bus_Tracking)
install(PROGRAMS
  scripts/test_plot.py
  scripts/test_plot1.py
  scripts/test_pub.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # # the following line skips the linter which checks for copyrights
  # # uncomment the line when a copyright and license is not present in all source files
  # # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # uncomment the line when this package is not in a git repo
  # # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
