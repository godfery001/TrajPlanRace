
cmake_minimum_required(VERSION 2.8.3)
project(cav_control)

add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS 
	cav_msgs
	roscpp 
	rospy 
	std_msgs)

catkin_package(CATKIN_DEPENDS  cav_msgs)


## Build
include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} )

include_directories(
	${PROJECT_SOURCE_DIR}/include
	src/rosNode
	src/path_track
	src/speed_track
    )


## src
file(GLOB SRC "src/*.cpp")
file(GLOB sub_SRC "src/*/*.cpp")

add_executable(${PROJECT_NAME} ${SRC} ${sub_SRC} ${MOCSrcs} ${UISrcs})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


## Install 
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
