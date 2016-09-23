PROJECT(threed_odometry)
cmake_minimum_required(VERSION 2.6)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(threed_odometryBase)

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

