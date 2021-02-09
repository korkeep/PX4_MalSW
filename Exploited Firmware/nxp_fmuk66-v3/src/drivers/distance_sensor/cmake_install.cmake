# Install script for directory: /home/kss/PX4/src/drivers/distance_sensor

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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/cm8jl65/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/leddar_one/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/ll40ls/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/ll40ls_pwm/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/mappydot/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/mb12xx/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/pga460/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/lightware_laser_i2c/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/lightware_laser_serial/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/srf02/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/teraranger/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/tfmini/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/ulanding_radar/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/vl53l0x/cmake_install.cmake")
  include("/home/kss/PX4/build/nxp_fmuk66-v3_default/src/drivers/distance_sensor/vl53l1x/cmake_install.cmake")

endif()

