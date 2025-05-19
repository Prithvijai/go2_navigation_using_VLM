# Install script for directory: /media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/install/robot_control")
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

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control/" TYPE DIRECTORY FILES
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/launch"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_control" TYPE PROGRAM FILES
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/robot_controller_gazebo.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/ramped_joypad.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/launch/robot_control.launch.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_control/RoboticsUtilities" TYPE PROGRAM FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RoboticsUtilities/Transformations.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_control/RobotController" TYPE PROGRAM FILES
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/CrawlGaitController.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/GaitController.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/PIDController.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/RobotController.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/StandController.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/StateCommand.py"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/RobotController/TrotGaitController.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_control/InverseKinematics" TYPE PROGRAM FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/scripts/InverseKinematics/robot_IK.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/robot_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/robot_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control/environment" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control/environment" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_index/share/ament_index/resource_index/packages/robot_control")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control/cmake" TYPE FILE FILES
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_core/robot_controlConfig.cmake"
    "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/ament_cmake_core/robot_controlConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_control" TYPE FILE FILES "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/src/robot_control/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/media/saitama/Games1/Documents_ubuntu/perception_project/go2_ros2_nav/build/robot_control/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
