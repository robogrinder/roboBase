
# cheeck the cmake version, if you cmake version is not 3.5, change to your current version.

cmake_minimum_required(VERSION 3.5)
project(RobotBase)

# To check the library dependence including glog, OpenCV, Boost, Threads, RapidJSON, and local directory of the project
find_package(glog REQUIRED)
find_package(OpenCV 4.3 REQUIRED )
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)
find_package(RapidJSON)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../)

# add the file that the project needs to be complied
add_executable(RobotBase
        RobotBase/main.cpp
        RobotBase/vision/bigbuff/BigbuffDetection.cpp
        RobotBase/vision/autoAim/autoAim.h
        RobotBase/vision/autoAim/autoAim.cpp
        RobotBase/vision/autoAim/armor.cpp
        RobotBase/vision/autoAim/armor.h
        RobotBase/vision/control.h
        RobotBase/record_video/recording_main.cpp
        RobotBase/record_video/recording_main.h
        RobotBase/vision/cam/Daheng.cpp
        RobotBase/vision/cam/Daheng.cpp
        RobotBase/Robogrinder_SDK/serial_port.cpp
        RobotBase/Robogrinder_SDK/serial_port.h
        RobotBase/vision/bigbuff/pred_algrsm.cpp
        RobotBase/vision/bigbuff/pred_algrsm.h
        RobotBase/common.h
        RobotBase/ThreadManagement/Thread_management.cpp
        RobotBase/ThreadManagement/Thread_management.h)

# The following part are linking the library into the binary file
link_directories(../)
target_link_libraries(RobotBase gxiapi) # library file of camera
target_link_libraries(RobotBase glog)
target_link_libraries(RobotBase ${OpenCV_LIBS})
target_link_libraries(RobotBase ${Boost_LIBRARIES})
target_link_libraries (RobotBase ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(RobotBase ${CMAKE_THREAD_LIBS_INIT})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")


# depend on the platform, we use the different compile strategy
# in the Jetson Nano, we use the 'Release' flag to compile the code to improve the code running efficiency
# in the VM, we use the 'debug' flag to compile the code to implement and debug the code
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message(STATUS "The current platform is arm")
    SET(CMAKE_BUILD_TYPE Release)
    message(STATUS "CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}' is defined")
    message(STATUS "The release flag is ${CMAKE_CXX_FLAGS_RELEASE}")
else ()
    message(STATUS "The current platform is x86_64")
    SET(CMAKE_BUILD_TYPE Debug)
    message(STATUS "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} is defined")
endif ()
