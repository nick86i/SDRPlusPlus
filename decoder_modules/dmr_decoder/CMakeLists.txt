cmake_minimum_required(VERSION 3.13)
project(dmr_decoder)

set(SDRPP_DSDCC_ROOT "" CACHE PATH "Use an existing patched DSDcc source tree instead of downloading it")
set(SDRPP_MBELIB_ROOT "" CACHE PATH "Use an existing mbelib source tree instead of downloading it")

if (SDRPP_DSDCC_ROOT)
    set(DSDCC_ROOT "${SDRPP_DSDCC_ROOT}")
elseif (EXISTS "${CMAKE_SOURCE_DIR}/../dsdcc/dsd_decoder.cpp")
    set(DSDCC_ROOT "${CMAKE_SOURCE_DIR}/../dsdcc")
else()
    include(FetchContent)
    find_package(Git REQUIRED)
    FetchContent_Declare(sdrpp_dsdcc
        GIT_REPOSITORY https://github.com/f4exb/dsdcc.git
        GIT_TAG f27b32d2df131ae3a376fe72d3fb880ae1f9ede1
        GIT_SHALLOW FALSE
        PATCH_COMMAND "${GIT_EXECUTABLE}" apply --whitespace=nowarn
            "${CMAKE_SOURCE_DIR}/third_party/patches/dsdcc-dmr-data.patch"
    )
    FetchContent_GetProperties(sdrpp_dsdcc)
    if (NOT sdrpp_dsdcc_POPULATED)
        FetchContent_Populate(sdrpp_dsdcc)
    endif()
    set(DSDCC_ROOT "${sdrpp_dsdcc_SOURCE_DIR}")
endif()

if (SDRPP_MBELIB_ROOT)
    set(MBELIB_ROOT "${SDRPP_MBELIB_ROOT}")
elseif (EXISTS "${CMAKE_SOURCE_DIR}/../mbelib/mbelib.h")
    set(MBELIB_ROOT "${CMAKE_SOURCE_DIR}/../mbelib")
else()
    include(FetchContent)
    FetchContent_Declare(sdrpp_mbelib
        GIT_REPOSITORY https://github.com/szechyjs/mbelib.git
        GIT_TAG 9a04ed5c78176a9965f3d43f7aa1b1f5330e771f
        GIT_SHALLOW FALSE
    )
    FetchContent_GetProperties(sdrpp_mbelib)
    if (NOT sdrpp_mbelib_POPULATED)
        FetchContent_Populate(sdrpp_mbelib)
    endif()
    set(MBELIB_ROOT "${sdrpp_mbelib_SOURCE_DIR}")
endif()

if (NOT EXISTS "${DSDCC_ROOT}/dsd_decoder.cpp" OR NOT EXISTS "${MBELIB_ROOT}/mbelib.h")
    message(FATAL_ERROR "DMR decoder dependencies are missing. Expected ../dsdcc and ../mbelib beside the SDR++ source tree.")
endif()

file(GLOB MBELIB_SOURCES "${MBELIB_ROOT}/*.c")
add_library(dmr_mbelib STATIC ${MBELIB_SOURCES})
target_include_directories(dmr_mbelib PUBLIC "${MBELIB_ROOT}")
target_compile_definitions(dmr_mbelib PRIVATE _USE_MATH_DEFINES)

set(DSDCC_SOURCES
    descramble.cpp dmr.cpp dsd_decoder.cpp dsd_filters.cpp dsd_logger.cpp
    dsd_mbe.cpp dsd_opts.cpp dsd_state.cpp dsd_symbol.cpp dsd_sync.cpp
    dstar.cpp ysf.cpp dpmr.cpp nxdn.cpp nxdnconvolution.cpp nxdncrc.cpp
    nxdnmessage.cpp p25p1_heuristics.cpp dsd_upsample.cpp fec.cpp viterbi.cpp
    viterbi3.cpp viterbi5.cpp crc.cpp pn.cpp mbefec.cpp locator.cpp
    phaselock.cpp timeutil.cpp
)
list(TRANSFORM DSDCC_SOURCES PREPEND "${DSDCC_ROOT}/")
add_library(dmr_dsdcc STATIC ${DSDCC_SOURCES})
target_include_directories(dmr_dsdcc PUBLIC "${DSDCC_ROOT}" "${MBELIB_ROOT}")
target_compile_definitions(dmr_dsdcc PUBLIC dsdcc_STATIC DSD_USE_MBELIB)
target_link_libraries(dmr_dsdcc PUBLIC dmr_mbelib)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c")
include(${SDRPP_MODULE_CMAKE})
target_include_directories(dmr_decoder PRIVATE "src/" "${DSDCC_ROOT}" "${MBELIB_ROOT}")
target_compile_definitions(dmr_decoder PRIVATE NOMINMAX)
target_link_libraries(dmr_decoder PRIVATE dmr_dsdcc)
