cmake_minimum_required(VERSION 2.8.3)
project(stereo_camera_subscriber)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	image_geometry
	image_transport
	sensor_msgs)

find_package(Boost REQUIRED COMPONENTS signals system)

catkin_package()

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} src/stereo_camera_subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})