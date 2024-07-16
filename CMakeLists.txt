cmake_minimum_required(VERSION 3.28)
project(TestProj)

set(CMAKE_CXX_STANDARD 17)
#set(OPENCV_SAMPLES_DATA_PATH "${PROJECT_SOURCE_DIR}/media") # doesn't seem to work

find_package( OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message("${OpenCV_INCLUDE_DIRS}")
find_package(depthai CONFIG REQUIRED)

#add_executable(TestProj main.cpp)
add_executable(TestProj main.cpp)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

set_property(TARGET TestProj PROPERTY CXX_STANDARD 17)

target_compile_definitions(TestProj PUBLIC SOURCE_DIR="${PROJECT_SOURCE_DIR}")

target_link_libraries(TestProj PRIVATE ${OpenCV_LIBS} depthai::opencv rerun_sdk)
