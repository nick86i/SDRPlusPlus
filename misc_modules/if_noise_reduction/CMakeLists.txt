cmake_minimum_required(VERSION 3.13)
project(if_noise_reduction)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c")
include(${SDRPP_MODULE_CMAKE})
target_include_directories(if_noise_reduction PRIVATE "src/" "../../decoder_modules/radio/src")
