cmake_minimum_required(VERSION 2.4.6)
include(/opt/ros/melodic/share/ros/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(CMAKE_CXX_STANDARD 17)
IF(NOT ROS_BUILD_TYPE)
   SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})
#set(MKLROOT "/home/firas/intel/compilers_and_libraries_2020.1.217/linux/mkl/")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -DMKL_ILP64 -m64 -I${MKLROOT}/include -O3 -g -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -DMKL_ILP64 -m64 -I${MKLROOT}/include -O3 -g -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
#include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/eigen-3.3.7)

find_package(Boost COMPONENTS system)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/../../../
        ${PROJECT_SOURCE_DIR}/../../../include
        ${Pangolin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/Utils_pnp
)





add_library(${PROJECT_NAME} SHARED
        ${PROJECT_SOURCE_DIR}/../../../src/System.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Tracking.cc
        ${PROJECT_SOURCE_DIR}/../../../src/LocalMapping.cc
        ${PROJECT_SOURCE_DIR}/../../../src/LoopClosing.cc
        ${PROJECT_SOURCE_DIR}/../../../src/ORBextractor.cc
        ${PROJECT_SOURCE_DIR}/../../../src/ORBmatcher.cc
        ${PROJECT_SOURCE_DIR}/../../../src/FrameDrawer.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Converter.cc
        ${PROJECT_SOURCE_DIR}/../../../src/MapPoint.cc
        ${PROJECT_SOURCE_DIR}/../../../src/KeyFrame.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Map.cc
        ${PROJECT_SOURCE_DIR}/../../../src/MapDrawer.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Optimizer.cc
        ${PROJECT_SOURCE_DIR}/../../../src/PnPsolver.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Frame.cc
        ${PROJECT_SOURCE_DIR}/../../../src/KeyFrameDatabase.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Sim3Solver.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Initializer.cc
        ${PROJECT_SOURCE_DIR}/../../../src/Viewer.cc
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/DualVar.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/LogDetHessianMatrix.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/PnpBarrierMethodSolver.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/PnpInput.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/PnpLocalSearch.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/PnpResult.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/PnpObjective.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/PnpProblemSolver.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/PnP/QuaternionVector.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/Utils_pnp/GeneralUtils.cpp
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/track_drone/Utils_pnp/Parsing.cpp
        )

set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
        )

# Node for monocular camera
rosbuild_add_executable(Mono
        src/ros_mono.cc
        )

target_link_libraries(Mono
        ${LIBS}
        )

# Node for monocular camera
rosbuild_add_executable(Mono_Load
        src/ros_mono_load.cc
        )

target_link_libraries(Mono_Load
        ${LIBS}
        )

# Node for monocular camera
rosbuild_add_executable(Mono_Save
        src/ros_mono_save.cc
        )

target_link_libraries(Mono_Save
        ${LIBS}
        )

# Node for stereo camera
rosbuild_add_executable(Stereo
        src/ros_stereo.cc
        )

target_link_libraries(Stereo
        ${LIBS}
        )

# Node for RGB-D camera
rosbuild_add_executable(RGBD
        src/ros_rgbd.cc
        )

target_link_libraries(RGBD
        ${LIBS}
        )
