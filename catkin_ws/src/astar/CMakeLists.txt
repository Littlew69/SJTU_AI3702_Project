########### Cmake version requirement ###########
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)

########### Project name ###########
project(astar)

########### Source of system lib ###########
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        tf
        )

catkin_package()

########### include file ###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

########### Produce the executable ###########
add_executable(${PROJECT_NAME} src/main.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
