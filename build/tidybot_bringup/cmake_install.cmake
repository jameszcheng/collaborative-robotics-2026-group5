# Install script for directory: /home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/install/tidybot_bringup")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/launch" TYPE DIRECTORY FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/config" TYPE DIRECTORY FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/rviz" TYPE DIRECTORY FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/rviz/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tidybot_bringup" TYPE PROGRAM FILES
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_base_sim.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_camera_sim.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_arms_sim.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_planner_sim.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_real_hardware.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_base_real.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_arms_real.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_grippers_real.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_pan_tilt_real.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_torque_hold.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_full_robot.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_tf.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/test_planner_real.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/diagnose_dynamixel.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/example_state_machine.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/trajectory_tracking.py"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts/pick_up_block_sim_copy.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/tidybot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/tidybot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/environment" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/environment" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_index/share/ament_index/resource_index/packages/tidybot_bringup")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup/cmake" TYPE FILE FILES
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_core/tidybot_bringupConfig.cmake"
    "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/ament_cmake_core/tidybot_bringupConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tidybot_bringup" TYPE FILE FILES "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/locobot/team-5-mon-600-730/collaborative-robotics-2026-group5/build/tidybot_bringup/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
