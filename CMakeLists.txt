cmake_minimum_required(VERSION 3.1...3.29)

project(
	ExampleDecoder
	VERSION 1.0
	LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -O3 -march=armv8-a -mcpu=apple-m1 -funroll-loops -flto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a -mcpu=apple-m1 -funroll-loops -flto")

# Add this line to fix the fuse.h error
add_compile_definitions(_FILE_OFFSET_BITS=64)

# ---------------------------------------------------------------
# 1) Tell pkg-config where to look for .pc files
# ---------------------------------------------------------------
set(ENV{PKG_CONFIG_PATH}
    "/usr/local/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# ---------------------------------------------------------------
# 2) Find libfuse2 via pkg-config
# ---------------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFUSE2 REQUIRED fuse)

include_directories(motioncam_decoder lib/include thirdparty ${LIBFUSE2_INCLUDE_DIRS})

link_directories(${LIBFUSE2_LIBRARY_DIRS})

# ---------------------------------------------------------------
# 3) Our library
# ---------------------------------------------------------------
add_library(motioncam_decoder lib/Decoder.cpp lib/RawData.cpp lib/RawData_Legacy.cpp)
set_property(TARGET motioncam_decoder PROPERTY POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------
# 4) Our example executable
# ---------------------------------------------------------------
add_executable(example example.cpp)

target_link_libraries(example PRIVATE motioncam_decoder ${LIBFUSE2_LIBRARIES})
