

cmake_minimum_required(VERSION 3.10.0)
project(cav_traj_gen)

add_compile_options(-std=c++11)


find_package( catkin REQUIRED COMPONENTS 
    cav_msgs
    roscpp
    rospy
    std_msgs)

catkin_package(CATKIN_DEPENDS cav_msgs)

#GUI
SET(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Quick Gui OpenGL)
find_package(OpenGL REQUIRED)

QT5_WRAP_UI(UISrcs src/GUI/gui.ui)
QT5_WRAP_CPP(MOCSrcs src/GUI/gui.h src/GUI/widgets/3d_display/display_3d.h src/GUI/widgets/battery/battery.h)


# Build
include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${CMAKE_CURRENT_BINARY_DIR})

include_directories( 
    ${QT_QTCORE_INCLUDE_DIR} 
    ${QT_QTGUI_INCLUDE_DIR} 
    ${QT_QTOPENGL_INCLUDE_DIR})

include_directories(
	include
    include/math
	${PROJECT_SOURCE_DIR}/include
	src/GUI
    src/GUI/widgets
    src/GUI/widgets/battery
    src/GUI/widgets/3d_display
	src/rosNode
    src/traj_generator/include
    src/traj_generator_template/include
    src/path_slicer
    )


## src
file(GLOB proj_SRC "src/*.cpp" "src/*.c")
file(GLOB proj_SRC_sub "src/*/*.cpp" "src/*/*.c" "src/*/*/*.cpp" "src/*/*/*/*.cpp")


add_executable(${PROJECT_NAME} ${proj_SRC} ${proj_SRC_sub}  ${MOCSrcs} ${UISrcs} )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARY}  Qt5::Core Qt5::Widgets Qt5::Quick Qt5::Gui Qt5::OpenGL)


## Install
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
