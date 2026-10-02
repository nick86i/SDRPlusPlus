cmake_minimum_required(VERSION 3.13)
project(drm_decoder)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.c")

include(${SDRPP_MODULE_CMAKE})

target_include_directories(drm_decoder PRIVATE "src/")

set(SDRPP_DREAM_ROOT "" CACHE PATH "Use an existing patched Dream source tree instead of downloading it")
if (SDRPP_DREAM_ROOT)
    set(DREAM_ROOT "${SDRPP_DREAM_ROOT}")
elseif (EXISTS "${CMAKE_SOURCE_DIR}/../dream/src/DrmReceiver.cpp")
    # Preserve the existing developer layout. A clean clone without this
    # sibling directory uses the pinned, patched upstream source below.
    set(DREAM_ROOT "${CMAKE_SOURCE_DIR}/../dream")
else()
    include(FetchContent)
    find_package(Git REQUIRED)
    FetchContent_Declare(sdrpp_dream
        GIT_REPOSITORY https://github.com/wwek/dream.git
        GIT_TAG fddf0cc69d0085d864d35e43e761de76104cfb17
        GIT_SHALLOW FALSE
        PATCH_COMMAND "${GIT_EXECUTABLE}" apply --whitespace=nowarn
            "${CMAKE_SOURCE_DIR}/third_party/patches/dream-embedded-sdrpp.patch"
    )
    FetchContent_GetProperties(sdrpp_dream)
    if (NOT sdrpp_dream_POPULATED)
        FetchContent_Populate(sdrpp_dream)
    endif()
    set(DREAM_ROOT "${sdrpp_dream_SOURCE_DIR}")
endif()

if (NOT EXISTS "${DREAM_ROOT}/src/DrmReceiver.cpp")
    message(FATAL_ERROR "Dream source was not found at: ${DREAM_ROOT}")
endif()
file(GLOB_RECURSE DREAM_CORE_SOURCES CONFIGURE_DEPENDS
    "${DREAM_ROOT}/src/*.cpp"
    "${DREAM_ROOT}/src/*.c")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/(GUI-QT|main-Qt|util-QT|android|linux|mac)/")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/main\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/DrmTransmitter\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/DrmSimulation\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/ctransmitdata\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/windows/Sound\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/MDI/PacketSocketNull\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/sound/(drm_portaudio|drm_pulseaudio)\\.cpp$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/sound/pa_ringbuffer\\.c$")
list(FILTER DREAM_CORE_SOURCES EXCLUDE REGEX "/util/Hamlib\\.cpp$")

add_library(dream_core STATIC ${DREAM_CORE_SOURCES})
target_include_directories(dream_core PUBLIC "${DREAM_ROOT}/src")
target_include_directories(dream_core PRIVATE "${DREAM_ROOT}")
target_compile_definitions(dream_core PRIVATE
    NOMINMAX DREAM_EMBEDDED _CRT_SECURE_NO_WARNINGS HAVE_STDINT_H HAVE_LIBZ)
find_package(FFTW3 CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
find_package(fdk-aac CONFIG REQUIRED)
find_path(SPEEXDSP_INCLUDE_DIR speex/speex_resampler.h REQUIRED)
find_library(SPEEXDSP_LIBRARY NAMES speexdsp REQUIRED)
target_include_directories(dream_core PRIVATE "${SPEEXDSP_INCLUDE_DIR}")
target_compile_definitions(dream_core PRIVATE HAVE_LIBFDK_AAC)
target_link_libraries(dream_core PUBLIC FFTW3::fftw3 ZLIB::ZLIB FDK-AAC::fdk-aac "${SPEEXDSP_LIBRARY}" ws2_32 winmm)

target_link_libraries(drm_decoder PRIVATE dream_core)
