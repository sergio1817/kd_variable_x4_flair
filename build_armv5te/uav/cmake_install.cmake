# Install script for directory: /home/ateverz/Documents/projects/kd_variable_x4_flair/uav

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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_rt" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_rt")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_rt"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_rt")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable" TYPE EXECUTABLE FILES "/home/ateverz/Documents/projects/kd_variable_x4_flair/build_armv5te/uav/kd_variable_rt")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_nrt" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_nrt")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_nrt"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/kd_variable_nrt")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable" TYPE EXECUTABLE FILES "/home/ateverz/Documents/projects/kd_variable_x4_flair/build_armv5te/uav/kd_variable_nrt")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/ctrl1_minidrones.sh")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/uav/resources/armv5te/ctrl1_minidrones.sh" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" armv5te "/" kd_variable "/" ctrl1_minidrones.sh " (file already exists)")
                endif()
            
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable/ctrl1_minidrones.xml")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/uav/resources/armv5te/ctrl1_minidrones.xml" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/armv5te/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" armv5te "/" kd_variable "/" ctrl1_minidrones.xml " (file already exists)")
                endif()
            
endif()

