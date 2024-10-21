cmake_minimum_required(VERSION 3.12)
project(foxglove)

add_definitions(-w)
add_definitions(-DCOMPILEDWITHC17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(Eigen3 3.4 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Protobuf 3.21 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
message("PROTOBUF_INCLUDE_DIRS", ${PROTOBUF_INCLUDE_DIRS})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

list(APPEND common_thirdparty_libs
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

add_subdirectory(src)
add_subdirectory(pybind)