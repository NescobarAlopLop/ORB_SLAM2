cmake_minimum_required(VERSION 2.8.4)
project(track_drone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(sample_dir ${CMAKE_CURRENT_SOURCE_DIR}/)
set(MKLROOT "/home/firas/intel/compilers_and_libraries_2020.1.217/linux/mkl/")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ltbb -D__STDC_CONSTANT_MACROS")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -std=c++11 -pthread")
#set(target firas_)
find_package( OpenCV 3.2.0 REQUIRED )
set(sample_pnplib
        ${sample_dir}motion_vectors.cpp
        ${sample_dir}Utils.cpp
        ${sample_dir}Product.cpp
        ${sample_dir}PnPProblem.cpp
        ${sample_dir}RobustMatcher.cpp
        ${sample_dir}frame.cpp
        ${sample_dir}Utils.cpp
        ${sample_dir}Projection.cpp
        ${sample_dir}PnP/PnpInput.cpp
        ${sample_dir}Utils_pnp/Parsing.cpp
        ${sample_dir}PnP/PnpObjective.cpp
        ${sample_dir}PnP/PnpProblemSolver.cpp
        ${sample_dir}PnP/DualVar.cpp
        ${sample_dir}PnP/PnpBarrierMethodSolver.cpp
        ${sample_dir}Utils_pnp/GeneralUtils.cpp
        ${sample_dir}PnP/PnpResult.cpp
        ${sample_dir}PnP/PnpLocalSearch.cpp
        ${sample_dir}PnP/QuaternionVector.cpp
        )
ocv_include_modules_recurse(${OPENCV_CPP_SAMPLES_REQUIRED_DEPS})

add_executable(track_drone main.cpp ${sample_pnplib})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(Utils_pnp PnP)
include_directories(eigen-3.3.7)
include_directories(-Idependencies/include -Ldependencies/lib)
target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} stdc++fs -ltbb -lstdc++ -lpthread -lm -ldl
        -lswscale -lavformat -lavcodec -lswresample -lavutil -lpthread -lz -lc -lrt)
target_link_libraries( track_drone ${OpenCV_LIBS} stdc++fs -ltbb -lstdc++ -lpthread -lm -ldl -lswscale -lavformat -lavcodec -lswresample -lavutil -lpthread -lz -lc -lrt)