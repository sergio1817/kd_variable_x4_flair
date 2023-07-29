# Install script for directory: /home/ateverz/Documents/projects/kd_variable_x4_flair/simulator

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
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libudt.so")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libudt.so" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libudt.so " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libudt.so.4.11")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libudt.so.4.11" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libudt.so.4.11 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libvrpn.so")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libvrpn.so" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libvrpn.so " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libvrpn.so.7.31")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libvrpn.so.7.31" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libvrpn.so.7.31 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libvrpn.so.7.33")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libvrpn.so.7.33" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libvrpn.so.7.33 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libFileLib.so")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libFileLib.so" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libFileLib.so " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libFileLib.so.1")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libFileLib.so.1" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libFileLib.so.1 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libFileLib.so.1.0")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libFileLib.so.1.0" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libFileLib.so.1.0 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libIrrlicht.so")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libIrrlicht.so" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libIrrlicht.so " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libIrrlicht.so.1.8")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libIrrlicht.so.1.8" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libIrrlicht.so.1.8 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libIrrlicht.so.1.8.4")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libIrrlicht.so.1.8.4" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libIrrlicht.so.1.8.4 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libquat.so")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libquat.so" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libquat.so " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libquat.so.7.31")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libquat.so.7.31" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libquat.so.7.31 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                        if (NOT EXISTS "/home/ateverz/flair/flair-install/lib/core2-64/libquat.so.7.33")
                            file(INSTALL "/opt/robomap3/2.1.3/core2-64/sysroots/core2-64-poky-linux/usr/lib/libquat.so.7.33" DESTINATION "/home/ateverz/flair/flair-install/lib/core2-64" USE_SOURCE_PERMISSIONS)
                        else()
                            message("-- Not installing " /home/ateverz/flair "/flair-install/lib/" core2-64 "/" libquat.so.7.33 " (file already exists)")
                        endif()
                    
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_rt" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_rt")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_rt"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_rt")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" TYPE EXECUTABLE FILES "/home/ateverz/Documents/projects/kd_variable_x4_flair/build/simulator/kd_variable_simulator_rt")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_nrt" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_nrt")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_nrt"
         RPATH "")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/kd_variable_simulator_nrt")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" TYPE EXECUTABLE FILES "/home/ateverz/Documents/projects/kd_variable_x4_flair/build/simulator/kd_variable_simulator_nrt")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/setup_x4.xml")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/simulator/resources/core2-64/setup_x4.xml" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" core2-64 "/" kd_variable "/" setup_x4.xml " (file already exists)")
                endif()
            
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/simulator_x4.sh")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/simulator/resources/core2-64/simulator_x4.sh" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" core2-64 "/" kd_variable "/" simulator_x4.sh " (file already exists)")
                endif()
            
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/simulator_x4.xml")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/simulator/resources/core2-64/simulator_x4.xml" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" core2-64 "/" kd_variable "/" simulator_x4.xml " (file already exists)")
                endif()
            
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/simulator_x8.sh")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/simulator/resources/core2-64/simulator_x8.sh" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" core2-64 "/" kd_variable "/" simulator_x8.sh " (file already exists)")
                endif()
            
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
                if (NOT EXISTS "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable/simulator_x8.xml")
                    file(INSTALL "/home/ateverz/Documents/projects/kd_variable_x4_flair/simulator/resources/core2-64/simulator_x8.xml" DESTINATION "/home/ateverz/flair/flair-install/bin/demos/core2-64/kd_variable" USE_SOURCE_PERMISSIONS)
                else()
                    message("-- Not installing " /home/ateverz/flair "/flair-install/bin/demos/" core2-64 "/" kd_variable "/" simulator_x8.xml " (file already exists)")
                endif()
            
endif()

