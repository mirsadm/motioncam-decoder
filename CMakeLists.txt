cmake_minimum_required(VERSION 3.1...3.29)

project(
	ExampleDecoder
	VERSION 1.0
	LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(motioncam_decoder lib/include thirdparty)

add_library(motioncam_decoder lib/Decoder.cpp lib/RawData.cpp)

add_executable(example example.cpp)

target_link_libraries(example PRIVATE motioncam_decoder)

if (MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()
