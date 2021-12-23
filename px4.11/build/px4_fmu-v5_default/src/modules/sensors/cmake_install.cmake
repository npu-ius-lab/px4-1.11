# Install script for directory: /home/dtpmh/PX4-Autopilot_1.11/src/modules/sensors

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
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/modules/sensors/data_validator/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/modules/sensors/sensor_calibration/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/modules/sensors/vehicle_acceleration/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/modules/sensors/vehicle_angular_velocity/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/modules/sensors/vehicle_air_data/cmake_install.cmake")
  include("/home/dtpmh/PX4-Autopilot_1.11/build/px4_fmu-v5_default/src/modules/sensors/vehicle_imu/cmake_install.cmake")

endif()

