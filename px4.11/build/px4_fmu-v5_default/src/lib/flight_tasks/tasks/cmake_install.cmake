# Install script for directory: /home/dtpmh/PX4-Autopilot_1.11/src/lib/flight_tasks/tasks

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/FlightTask/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Utility/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Auto/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/AutoMapper/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/ManualAltitude/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/ManualAltitudeSmooth/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/ManualAltitudeSmoothVel/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/ManualPosition/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/ManualPositionSmooth/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/ManualPositionSmoothVel/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/AutoLineSmoothVel/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/AutoFollowMe/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Offboard/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Failsafe/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Descend/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Transition/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/lib/flight_tasks/tasks/Orbit/cmake_install.cmake")

endif()

