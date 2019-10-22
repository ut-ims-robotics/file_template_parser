cmake_minimum_required(VERSION 2.8.3)
project(file_template_parser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roslib
  roscpp
  std_msgs
)

find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ft_parser
  CATKIN_DEPENDS roscpp std_msgs roslib cmake_modules
  DEPENDS TinyXML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

# Create file template parser library
add_library(ft_parser STATIC 
  src/template_container.cpp
  src/template_management_tools.cpp
)
add_dependencies(ft_parser ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(ft_parser ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

# Create a testing node
add_executable(template_import_test src/template_import_test.cpp)
add_dependencies(template_import_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(template_import_test ft_parser ${catkin_LIBRARIES} )
