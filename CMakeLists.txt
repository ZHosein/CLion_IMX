cmake_minimum_required(VERSION 3.28)
project(TestProj)

set(CMAKE_CXX_STANDARD 17)

# Linking boost (for GTSAM)
find_package(Boost 1.65.0 REQUIRED system filesystem serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

# Adding path to Eigen to includes (for GTSAM)
include_directories(/usr/local/include/gtsam/3rdparty/)
include_directories(/usr/local/include/gtsam/3rdparty/Eigen)


find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${GTSAM_INCLUDE_DIR})


add_executable(TestProj main.cpp)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

set_property(TARGET TestProj PROPERTY CXX_STANDARD 17)

target_compile_definitions(TestProj PUBLIC SOURCE_DIR="${PROJECT_SOURCE_DIR}")

target_link_libraries(TestProj PRIVATE
        ${OpenCV_LIBS} depthai::opencv rerun_sdk ${Boost_LIBRARIES} metis-gtsam gtsam gtsam_unstable nlohmann_json::nlohmann_json
)
