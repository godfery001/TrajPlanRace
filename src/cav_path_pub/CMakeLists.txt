

cmake_minimum_required(VERSION 3.10.0)
project(cav_path_pub)

add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS 
    cav_msgs
    roscpp
    rospy
    std_msgs)

file(GLOB SCRIPTS "scripts/*.py")

# catkin_python_setup()

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs cav_msgs)



catkin_install_python(PROGRAMS ${SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
