cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_race)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp std_msgs turtlesim)
find_package(PkgConfig REQUIRED)
catkin_package(

  CATKIN_DEPENDS geometry_msgs roscpp std_msgs turtlesim 

)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
pkg_check_modules(CPPAD REQUIRED cppad)
pkg_check_modules(IPOPT REQUIRED ipopt>=3.12)

add_executable(Overtake src/Overtake.cpp)
target_link_libraries(Overtake ${catkin_LIBRARIES})
target_link_libraries(Overtake ${IPOPT_LIBRARIES})