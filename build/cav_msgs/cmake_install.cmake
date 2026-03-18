# Install script for directory: /home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/siegf/Downloads/path_planning_test/TrajPlanRace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_msgs/msg" TYPE FILE FILES
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Control.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/VehicleState.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Gpybm.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/PlanedPath.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Point.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/RefPoint.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Goal.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/Obstacle.msg"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/msg/ObstacleVec.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_msgs/cmake" TYPE FILE FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/build/cav_msgs/catkin_generated/installspace/cav_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/devel/include/cav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/devel/share/roseus/ros/cav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/devel/share/common-lisp/ros/cav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/devel/share/gennodejs/ros/cav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/siegf/Downloads/path_planning_test/TrajPlanRace/devel/lib/python3/dist-packages/cav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/devel/lib/python3/dist-packages/cav_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/build/cav_msgs/catkin_generated/installspace/cav_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_msgs/cmake" TYPE FILE FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/build/cav_msgs/catkin_generated/installspace/cav_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_msgs/cmake" TYPE FILE FILES
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/build/cav_msgs/catkin_generated/installspace/cav_msgsConfig.cmake"
    "/home/siegf/Downloads/path_planning_test/TrajPlanRace/build/cav_msgs/catkin_generated/installspace/cav_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cav_msgs" TYPE FILE FILES "/home/siegf/Downloads/path_planning_test/TrajPlanRace/src/cav_msgs/package.xml")
endif()

