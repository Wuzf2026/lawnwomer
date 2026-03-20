# Install script for directory: /home/norco/Desktop/lawnwomer_ws/src/orbbec_camera

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/norco/Desktop/lawnwomer_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE PROGRAM FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE PROGRAM FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/setup.bash;/home/norco/Desktop/lawnwomer_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/setup.bash"
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/setup.sh;/home/norco/Desktop/lawnwomer_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/setup.sh"
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/setup.zsh;/home/norco/Desktop/lawnwomer_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/setup.zsh"
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/setup.fish;/home/norco/Desktop/lawnwomer_ws/install/local_setup.fish")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/setup.fish"
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/local_setup.fish"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/norco/Desktop/lawnwomer_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/norco/Desktop/lawnwomer_ws/install" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/msg" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/msg/DeviceInfo.msg"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/msg/Extrinsics.msg"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/msg/Metadata.msg"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/msg/IMUInfo.msg"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/msg/DeviceStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/srv" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/GetBool.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/SetBool.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/GetCameraInfo.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/GetCameraParams.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/GetDeviceInfo.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/GetInt32.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/SetFilter.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/GetString.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/SetInt32.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/SetString.srv"
    "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/srv/SetArrays.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/orbbec_camera-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/include/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/share/roseus/ros/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/share/common-lisp/ros/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/share/gennodejs/ros/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/python3/dist-packages/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/python3/dist-packages/orbbec_camera")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/orbbec_camera.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/orbbec_camera-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/cmake" TYPE FILE FILES
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/orbbec_cameraConfig.cmake"
    "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/orbbec_cameraConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE PROGRAM FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/common_benchmark_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE PROGRAM FILES "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/catkin_generated/installspace/service_benchmark_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so"
         RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/liborbbec_camera.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so"
         OLD_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64:/opt/ros/noetic/lib:"
         NEW_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so"
         RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/liborbbec_camera_nodelet.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so"
         OLD_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64:/opt/ros/noetic/lib:"
         NEW_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/liborbbec_camera_nodelet.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node"
         RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/orbbec_camera/orbbec_camera_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node"
         OLD_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64:/opt/ros/noetic/lib:/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib:"
         NEW_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/orbbec_camera_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node"
         RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/orbbec_camera/list_devices_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node"
         OLD_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64:/opt/ros/noetic/lib:/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib:"
         NEW_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_devices_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node"
         RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/orbbec_camera/list_depth_work_mode_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node"
         OLD_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64:/opt/ros/noetic/lib:/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib:"
         NEW_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_depth_work_mode_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node"
         RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera" TYPE EXECUTABLE FILES "/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib/orbbec_camera/list_camera_profile_mode_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node"
         OLD_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64:/opt/ros/noetic/lib:/home/norco/Desktop/lawnwomer_ws/devel/.private/orbbec_camera/lib:"
         NEW_RPATH ":/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orbbec_camera/list_camera_profile_mode_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera/SDK/lib" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orbbec_camera/" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/include")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orbbec_camera/" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/launch")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/nodelet_plugins.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/orbbec_camera" TYPE FILE FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/LICENSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/" TYPE DIRECTORY FILES "/home/norco/Desktop/lawnwomer_ws/src/orbbec_camera/SDK/lib/arm64/" FILES_MATCHING REGEX "/[^/]*\\.so$" REGEX "/[^/]*\\.so\\.[^/]*$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/norco/Desktop/lawnwomer_ws/build/orbbec_camera/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
