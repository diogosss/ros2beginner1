cmake_minimum_required(VERSION 3.8)
project(components_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#para encontrar los hpp files
include_directories(include)

add_executable(manual_composition src/manual_composition.cpp src/node1.cpp src/node2.cpp) #no olvidar la dirección de los node1.cpp node2.cpp
ament_target_dependencies(manual_composition rclcpp) 



# instalar en algun ligar de projecty name
install(TARGETS
  manual_composition
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
