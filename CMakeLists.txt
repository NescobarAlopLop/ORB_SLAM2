cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)
set(CMAKE_CXX_STANDARD 17)
IF(NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
#set(MKLROOT "/home/firas/intel/compilers_and_libraries_2020.1.217/linux/mkl/")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -march=native")

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

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost  1.54.0  	     # Minimum or EXACT version e.g. 1.36.0
        REQUIRED             					# Fail with error if Boost is not found
        COMPONENTS serialization 				# Boost libraries by their canonical name
        )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}Thirdparty/track_drone
        ${PROJECT_SOURCE_DIR}Thirdparty/track_drone/PnP
        ${PROJECT_SOURCE_DIR}Thirdparty/track_drone/Utils_pnp
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        Thirdparty/track_drone/PnPProblem.cpp
        Thirdparty/track_drone/RobustMatcher.cpp
        Thirdparty/track_drone/PnP/PnpInput.cpp
        Thirdparty/track_drone/Utils_pnp/Parsing.cpp
        Thirdparty/track_drone/PnP/PnpObjective.cpp
        Thirdparty/track_drone/PnP/PnpProblemSolver.cpp
        Thirdparty/track_drone/PnP/DualVar.cpp
        Thirdparty/track_drone/PnP/PnpBarrierMethodSolver.cpp
        Thirdparty/track_drone/Utils_pnp/GeneralUtils.cpp
        Thirdparty/track_drone/PnP/PnpResult.cpp
        Thirdparty/track_drone/PnP/PnpLocalSearch.cpp
        Thirdparty/track_drone/PnP/QuaternionVector.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        ${Boost_LIBRARIES}
        )

set(sample_pnplib

        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnPProblem.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/RobustMatcher.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/PnpInput.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/Utils_pnp/Parsing.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/PnpObjective.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/PnpProblemSolver.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/DualVar.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/PnpBarrierMethodSolver.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/Utils_pnp/GeneralUtils.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/PnpResult.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/PnpLocalSearch.cpp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP/QuaternionVector.cpp
        )
ocv_include_modules_recurse(${OPENCV_CPP_SAMPLES_REQUIRED_DEPS})

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)
include_directories(
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/Utils_pnp
        ${PROJECT_SOURCE_DIR}/Thirdparty/track_drone/PnP
)
include_directories(
        -Idependencies/include
        -Ldependencies/lib
)

target_link_libraries(
        ${PROJECT_NAME}
        ${OpenCV_LIBS}
        stdc++fs -ltbb -lstdc++ -lpthread -lm -ldl -lswscale
        -lavformat -lavcodec -lswresample -lavutil -lpthread
        -lz -lc -lrt
)


# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)

add_executable(rgbd_tum
        Examples/RGB-D/rgbd_tum.cc ${sample_pnplib})
target_link_libraries(rgbd_tum ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

add_executable(stereo_kitti
        Examples/Stereo/stereo_kitti.cc ${sample_pnplib})
target_link_libraries(stereo_kitti ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(mono_tum
        Examples/Monocular/mono_tum.cc ${sample_pnplib})
target_link_libraries(mono_tum ${PROJECT_NAME})

add_executable(mono_kitti
        Examples/Monocular/mono_kitti.cc ${sample_pnplib})
target_link_libraries(mono_kitti ${PROJECT_NAME})

# Build tools
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary
        tools/bin_vocabulary.cc ${sample_pnplib})
target_link_libraries(bin_vocabulary ${PROJECT_NAME})