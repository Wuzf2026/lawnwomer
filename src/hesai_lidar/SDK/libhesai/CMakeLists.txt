# ----------------------------------------#
# Hesai Tech <http://www.hesaitech.com>  #
# ----------------------------------------#

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 14)
project(CommonModules)
if(POLICY CMP0074)
cmake_policy(SET CMP0074 NEW)
else()
cmake_policy(SET CMP0065 NEW)
endif()  

if(FIND_CUDA)
    find_package(CUDA )
endif()
if(${CUDA_FOUND})
    project(CommonModules LANGUAGES CXX CUDA)
	set(CUDA_SOURCE_PROPERTY_FORMAT OBJ)
	set(CUDA_SEPARABLE_COMPILATION ON)
	include_directories(${CUDA_INCLUDE_DIRS})
	set(CUDA_PROPAGATE_HOST_FLAGS OFF)
	set(CUDA_NVCC_FLAGS -arch=sm_61;-O3;-G;-g;-std=c++14)#根据具体GPU性能更改算力参数
	link_directories($ENV{CUDA_PATH}/lib/x64)
    set(WITH_UDPPARSERGPU ON CACHE BOOL "Build UdpParserGpu code")
else(${CUDA_FOUND})
	MESSAGE(STATUS "cuda not found!")
endif(${CUDA_FOUND})

set(CMAKE_BUILD_TYPE ON)

# Add each module's entry
set(WITH_LIDARBOARD ON CACHE BOOL "Build lidarboard code")
set(WITH_UDPPARSER ON CACHE BOOL "Build udp parser code")
set(WITH_PTCCLIENT ON CACHE BOOL "Build ptc client code")
if (WITH_LIDARBOARD AND WITH_UDPPARSER AND WITH_PTCCLIENT)
    set(WITH_LIDARLIB ON CACHE BOOL "Build lidar code")
endif()

find_package(Threads REQUIRED)  
add_definitions(-DQT_MESSAGELOGCONTEXT)

include_directories(
    ./
)

# Make each module an 'OBJECT' library, can be refered to by get_link_libraries
if(WITH_UDPPARSER)
    add_library(udpParser_lib 
        UdpParser/udp1_4_parser.cc
        )
    target_link_libraries(udpParser_lib
                        log_lib
                        platutils_lib)
    set_target_properties(udpParser_lib 
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
    target_include_directories(udpParser_lib PUBLIC   
        UdpParser
        ./
    ) 
    if(WITH_UDPPARSERGPU)
        add_library(udpParserGpu_lib 
            UdpParserGpu/src/udp_parser_gpu_kernel.cu
            UdpParserGpu/src/udp1_4_parser_gpu.cu
            )
        target_link_libraries(udpParserGpu_lib
            udpParser_lib)
        target_include_directories(udpParserGpu_lib PUBLIC   
            UdpParserGpu/include
            UdpParserGpu/src
            UdpParserGpu)
    endif(WITH_UDPPARSERGPU)
endif(WITH_UDPPARSER)

if(WITH_PTCCLIENT)
    add_library(ptcClient_lib 
        PtcClient/src/ptc_client.cc
        PtcClient/src/tcp_client.cc
        PtcParser/src/general_ptc_parser.cc
        PtcParser/src/ptc_1_0_parser.cc
        PtcParser/src/ptc_2_0_parser.cc
        PtcParser/ptc_parser.cc)
    target_link_libraries(ptcClient_lib
                        platutils_lib)
    set_target_properties(ptcClient_lib 
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
    target_include_directories(ptcClient_lib PUBLIC   
        PtcClient/include
        PtcParser/include
        PtcParser
    ) 
endif(WITH_PTCCLIENT)

if(WITH_LIDARBOARD)
    add_library(source_lib 
        Source/src/source.cc
        Source/src/pcap_source.cc
        Source/src/socket_source.cc
        Source/src/pcap_saver.cc
        )
    target_link_libraries(source_lib
        platutils_lib)
    set_target_properties(source_lib
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
    target_include_directories(source_lib PUBLIC   
        Source/include
        Container/include
        Container/src
    ) 
endif(WITH_LIDARBOARD)

# platutils lib
add_library(platutils_lib 
    ./src/plat_utils.cc
    ./src/auto_tick_count.cc)
target_link_libraries(platutils_lib
    log_lib)
set_target_properties(platutils_lib
    PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
target_include_directories(platutils_lib PUBLIC   
    ./include
)
# log lib
add_library(log_lib 
    Logger/src/logger.cc)
target_link_libraries(log_lib
    ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(log_lib
    PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
target_include_directories(log_lib PUBLIC   
    Logger/include
)

if (WITH_LIDARLIB)
    add_library(lidar_lib 
        lidar/lidar.cc)
    target_link_libraries(lidar_lib
        log_lib
        platutils_lib
        udpParser_lib
        source_lib
        ptcClient_lib
        ${CMAKE_THREAD_LIBS_INIT})
    if(WITH_UDPPARSERGPU)
        target_link_libraries(lidar_lib
            udpParserGpu_lib)
    endif(WITH_UDPPARSERGPU)
    set_target_properties(lidar_lib
        PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 14)
    target_include_directories(lidar_lib PUBLIC   
        lidar
        ./
    )
endif(WITH_LIDARLIB)

