cmake_minimum_required(VERSION 2.8)
project(orb_rgbd)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")

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

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

set(ORB_SLAM2_DIR "/home/cvlab/ORB_SLAM2/")
#set(Pangolin_INCLUDE_DIRS "/home/cvlab/Pangolin/include")
set(Pangolin_so "/home/cvlab/Pangolin/build/src/")

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
#set(Pangolin_INCLUDE_DIRS "/home/cvlab/Pangolin/include")
include_directories(
${ORB_SLAM2_DIR}
${ORB_SLAM2_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${Pangolin_so}
${PCL_INCLUDE_DIRS}
)

add_executable(orb_rgbd
orb_rgbd.cpp)

message(STATUS "Panglon libraries ${Pangolin_LIBRARIES}")
#message(STATUS "Panglon libs ${Pangolin_LIBS}")
target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM2_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_SLAM2_DIR}/Thirdparty/g2o/lib/libg2o.so
${ORB_SLAM2_DIR}/lib/libORB_SLAM2.so
)


add_executable(lines
lines.cpp)

target_link_libraries(lines ${PCL_LIBRARIES})

add_executable(planes
planes.cpp)

target_link_libraries(planes ${PCL_LIBRARIES})
