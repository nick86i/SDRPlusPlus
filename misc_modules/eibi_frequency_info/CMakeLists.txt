cmake_minimum_required(VERSION 3.13)
project(eibi_frequency_info)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c")
include(${SDRPP_MODULE_CMAKE})
target_include_directories(eibi_frequency_info PRIVATE "src/")
