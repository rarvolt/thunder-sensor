cmake_minimum_required(VERSION 3.15)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(thunder_detect)

set(CMAKE_CXX_STANDARD 14)

