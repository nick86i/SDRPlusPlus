cmake_minimum_required(VERSION 3.13)
project(dvbt2_decoder)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c")
include(${SDRPP_MODULE_CMAKE})

set(SDRPP_DVBT2_REFERENCE_ROOT "" CACHE PATH
    "Use an existing patched sdr_receiver_dvb_t2 source tree instead of downloading it")
if (SDRPP_DVBT2_REFERENCE_ROOT)
    set(DVBT2_REFERENCE_ROOT "${SDRPP_DVBT2_REFERENCE_ROOT}/src")
elseif (EXISTS "${CMAKE_SOURCE_DIR}/external/sdr_receiver_dvb_t2/src/DVB_T2/dvbt2_definition.cpp")
    set(DVBT2_REFERENCE_ROOT "${CMAKE_SOURCE_DIR}/external/sdr_receiver_dvb_t2/src")
else()
    include(FetchContent)
    find_package(Git REQUIRED)
    FetchContent_Declare(sdrpp_dvbt2_reference
        GIT_REPOSITORY https://github.com/Oleg-Malyutin/sdr_receiver_dvb_t2.git
        GIT_TAG 332e9704c29d16940a7322b9f49a11a6f5c04437
        GIT_SHALLOW FALSE
        PATCH_COMMAND "${GIT_EXECUTABLE}" apply --whitespace=nowarn
            "${CMAKE_SOURCE_DIR}/third_party/patches/sdr_receiver_dvb_t2-ldpc.patch"
    )
    FetchContent_GetProperties(sdrpp_dvbt2_reference)
    if (NOT sdrpp_dvbt2_reference_POPULATED)
        FetchContent_Populate(sdrpp_dvbt2_reference)
    endif()
    set(DVBT2_REFERENCE_ROOT "${sdrpp_dvbt2_reference_SOURCE_DIR}/src")
endif()

if (NOT EXISTS "${DVBT2_REFERENCE_ROOT}/DVB_T2/dvbt2_definition.cpp")
    message(FATAL_ERROR "DVB-T2 reference source was not found at: ${DVBT2_REFERENCE_ROOT}")
endif()

target_sources(dvbt2_decoder PRIVATE
    "${DVBT2_REFERENCE_ROOT}/DVB_T2/dvbt2_definition.cpp"
    "${DVBT2_REFERENCE_ROOT}/DVB_T2/pilot_generator.cpp"
    "${DVBT2_REFERENCE_ROOT}/DVB_T2/address_freq_deinterleaver.cpp"
)
target_include_directories(dvbt2_decoder PRIVATE "src/" "src/bch/include" "${DVBT2_REFERENCE_ROOT}" "${DVBT2_REFERENCE_ROOT}/DVB_T2")
target_compile_definitions(dvbt2_decoder PRIVATE NOMINMAX)
if (MSVC)
    # The DVB-T2 LDPC reference implementation expresses SIMD lanes as small
    # fixed loops.  Target the AVX2-capable x64 CPU so MSVC can vectorize those
    # saturated byte operations instead of executing every lane serially.
    target_compile_options(dvbt2_decoder PRIVATE /arch:AVX2)
endif()
find_package(FFTW3f CONFIG REQUIRED)
target_link_libraries(dvbt2_decoder PRIVATE FFTW3::fftw3f)
if (WIN32)
    target_link_libraries(dvbt2_decoder PRIVATE ws2_32)
endif()

add_executable(dvbt2_ldpc_replay EXCLUDE_FROM_ALL
    "tools/ldpc_replay.cpp"
    "src/bch/lib/bch.cpp"
    "src/bch/lib/gf.cpp"
)
target_include_directories(dvbt2_ldpc_replay PRIVATE
    "src/bch/include" "${DVBT2_REFERENCE_ROOT}" "${DVBT2_REFERENCE_ROOT}/DVB_T2"
)
target_compile_definitions(dvbt2_ldpc_replay PRIVATE NOMINMAX)
target_compile_features(dvbt2_ldpc_replay PRIVATE cxx_std_17)
if (MSVC)
    target_compile_options(dvbt2_ldpc_replay PRIVATE /arch:AVX2)
endif()
