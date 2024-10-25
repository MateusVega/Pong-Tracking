cmake_minimum_required(VERSION 3.29)
project(NicePong)

include(FetchContent)
set(BUILD_SHARED_LIBS OFF)

FETCHCONTENT_DECLARE(SFML GIT_REPOSITORY https://github.com/SFML/SFML.git GIT_TAG 2.5.1)
FETCHCONTENT_MAKEAVAILABLE(SFML)

set(OpenCV_DIR "C:/opencv/build")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libstdc++ -static-libgcc")

add_executable(NicePong main.cpp)

target_link_libraries(NicePong sfml-graphics sfml-window sfml-system ${OpenCV_LIBS})

include_directories(${OpenCV_INCLUDE_DIRS})
