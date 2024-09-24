cmake_minimum_required(VERSION 3.0.0)
project(yac)

# Compilation Settings
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra)
set(USE_ADDRESS_SANITIZER OFF)

# Address Sanitizer
if (USE_ADDRESS_SANITIZER)
  message(AUTHOR_WARNING "Building with address sanitizer - This will make executables 2x slower!")
  add_compile_options(
    -fsanitize=address
    -fno-omit-frame-pointer
  )
  link_libraries(
    -fsanitize=address
    -static-libasan # Use -static-libsan for Clang
  )
endif()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(yaml-cpp REQUIRED)
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")
message(STATUS "PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "Eigen3 Dir: ${Eigen3_DIR}")
message(STATUS "Eigen3 Include Dir: ${EIGEN3_INCLUDE_DIR}")

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)

# Build libyac
include_directories(lib)
add_library(
  yac
  lib/AprilGrid.cpp
  lib/CalibCamera.cpp
  lib/CalibData.cpp
  lib/CalibTarget.cpp
  lib/CalibView.cpp
  lib/CameraChain.cpp
  lib/CameraGeometry.cpp
  lib/CameraModel.cpp
  lib/Core.cpp
  lib/InertialError.cpp
  lib/ParamBlock.cpp
  lib/PoseLocalParameterization.cpp
  lib/ReprojectionError.cpp
  lib/ResidualBlock.cpp
  lib/SolvePnp.cpp
  lib/Sim.cpp
  lib/Timeline.cpp
)
set(DEPS yaml-cpp::yaml-cpp Eigen3::Eigen ${OpenCV_LIBS} apriltag::apriltag apriltags Ceres::ceres)
target_link_libraries(yac ${DEPS})

# Build unit-tests
link_directories(${CMAKE_PREFIX_PATH}/lib)
set(TEST_DEPS GTest::gtest_main ${PROJECT_NAME} ${DEPS})
set(TEST_DATA ${CMAKE_CURRENT_LIST_DIR}/tests/test_data)

add_executable(TestAprilGrid tests/TestAprilGrid.cpp)
add_compile_definitions(TestAprilGrid PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestAprilGrid ${TEST_DEPS})

add_executable(TestCalibCamera tests/TestCalibCamera.cpp)
add_compile_definitions(TestCalibCamera PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCalibCamera ${TEST_DEPS})

add_executable(TestCalibData tests/TestCalibData.cpp)
add_compile_definitions(TestCalibData PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCalibData ${TEST_DEPS})

add_executable(TestCameraChain tests/TestCameraChain.cpp)
add_compile_definitions(TestCameraChain PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCameraChain ${TEST_DEPS})

add_executable(TestCameraModel tests/TestCameraModel.cpp)
add_compile_definitions(TestCameraModel PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCameraModel ${TEST_DEPS})

add_executable(TestInertialError tests/TestInertialError.cpp)
add_compile_definitions(TestInertialError PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestInertialError ${TEST_DEPS})

add_executable(TestReprojectionError tests/TestReprojectionError.cpp)
add_compile_definitions(TestReprojectionError PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestReprojectionError ${TEST_DEPS})

add_executable(TestSim tests/TestSim.cpp)
add_compile_definitions(TestSim PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestSim ${TEST_DEPS})

add_executable(TestTimeline tests/TestTimeline.cpp)
add_compile_definitions(TestTimeline PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestTimeline ${TEST_DEPS})
