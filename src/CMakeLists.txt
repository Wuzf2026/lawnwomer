# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake
cmake_minimum_required(VERSION 3.5)
project(Project)

# 隔离缓存变量（避免跨包生效）
set(WITH_PTCS_USE ON CACHE BOOL "Use Ptcs" FORCE)
set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ standard" FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON CACHE BOOL "" FORCE)

# 避免全局编译选项覆盖
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" CACHE STRING "CXX flags" FORCE)

set(CATKIN_TOPLEVEL TRUE)

# search for catkin within the workspace
set(_cmd "catkin_find_pkg" "catkin" "${CMAKE_SOURCE_DIR}")
execute_process(COMMAND ${_cmd}
  RESULT_VARIABLE _res
  OUTPUT_VARIABLE _out
  ERROR_VARIABLE _err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
if(NOT _res EQUAL 0 AND NOT _res EQUAL 2)
  # searching for catkin resulted in an error
  string(REPLACE ";" " " _cmd_str "${_cmd}")
  message(FATAL_ERROR "Search for 'catkin' in workspace failed (${_cmd_str}): ${_err}")
endif()

# include catkin from workspace or via find_package()
if(_res EQUAL 0)
  set(catkin_EXTRAS_DIR "${CMAKE_SOURCE_DIR}/${_out}/cmake")
  # include all.cmake without add_subdirectory to let it operate in same scope
  include(${catkin_EXTRAS_DIR}/all.cmake NO_POLICY_SCOPE)
  add_subdirectory("${_out}")

else()
  # use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
  # or CMAKE_PREFIX_PATH from the environment
  if(NOT DEFINED CMAKE_PREFIX_PATH)
    if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      if(NOT WIN32)
        string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      else()
        set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      endif()
    endif()
  endif()

  # list of catkin workspaces
  set(catkin_search_path "")
  foreach(path ${CMAKE_PREFIX_PATH})
    if(EXISTS "${path}/.catkin")
      list(FIND catkin_search_path ${path} _index)
      if(_index EQUAL -1)
        list(APPEND catkin_search_path ${path})
      endif()
    endif()
  endforeach()

  # search for catkin in all workspaces
  set(CATKIN_TOPLEVEL_FIND_PACKAGE TRUE)
  find_package(catkin QUIET
    NO_POLICY_SCOPE
    PATHS ${catkin_search_path}
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  unset(CATKIN_TOPLEVEL_FIND_PACKAGE)

  if(NOT catkin_FOUND)
    message(FATAL_ERROR "find_package(catkin) failed. catkin was neither found in the workspace nor in the CMAKE_PREFIX_PATH. One reason may be that no ROS setup.sh was sourced before.")
  endif()
endif()

catkin_workspace()

# 仅 ROS1 编译，移除 isolated 编译（普通 catkin_make 即可）
# catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release