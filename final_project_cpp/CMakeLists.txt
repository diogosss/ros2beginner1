cmake_minimum_required(VERSION 3.8)
project(final_project_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_interfaces REQUIRED) 
find_package(turtlesim REQUIRED) 
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(turtle_controller_step3 src/turtle_controller_step3.cpp)
ament_target_dependencies(turtle_controller_step3 rclcpp my_robot_interfaces turtlesim rclcpp_action geometry_msgs rclcpp_lifecycle) 

# instalar en algun ligar de projecty name
install(TARGETS
  turtle_controller_step3
  DESTINATION lib/${PROJECT_NAME}
)

# PARA CREAR EL NODO
add_library(final_project_component SHARED src/turtle_controller.cpp)
ament_target_dependencies(
  final_project_component
  rclcpp
  my_robot_interfaces
  turtlesim
  rclcpp_action
  geometry_msgs
  rclcpp_lifecycle
  rclcpp_components
  )
rclcpp_components_register_nodes(final_project_component "final_project::TurtleController")

install(TARGETS
  final_project_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
