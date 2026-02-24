cmake_minimum_required(VERSION 3.8)
project(executors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_executable(single_threaded_executor src/single_threaded_executor.cpp)
ament_target_dependencies(single_threaded_executor rclcpp)

add_executable(multi_threaded_executor src/multi_threaded_executor.cpp)
ament_target_dependencies(multi_threaded_executor rclcpp)

install(TARGETS
  single_threaded_executor
  multi_threaded_executor
  DESTINATION lib/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
