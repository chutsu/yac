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
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(yaml-cpp REQUIRED)

# Build libyac
add_library(
  yac
  AprilGrid.cpp
  CalibData.cpp
  CalibTarget.cpp
  CameraChain.cpp
  CameraGeometry.cpp
  CameraModel.cpp
  Core.cpp
  ResidualBlock.cpp
)
set(DEPS yaml-cpp::yaml-cpp Eigen3::Eigen ${OpenCV_LIBS} apriltag::apriltag apriltags Ceres::ceres)
target_link_libraries(yac ${DEPS})

# Build unit-tests
link_directories(${CMAKE_PREFIX_PATH}/lib)
set(TEST_DEPS GTest::gtest_main ${PROJECT_NAME} ${DEPS})
set(TEST_DATA ${CMAKE_CURRENT_LIST_DIR}/test_data)

add_executable(TestAprilGrid TestAprilGrid.cpp)
add_compile_definitions(TestAprilGrid PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestAprilGrid ${TEST_DEPS})

add_executable(TestCalibData TestCalibData.cpp)
add_compile_definitions(TestCalibData PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCalibData ${TEST_DEPS})

add_executable(TestCameraResidual TestCameraResidual.cpp)
add_compile_definitions(TestCameraResidual PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCameraResidual ${TEST_DEPS})

add_executable(TestCameraModel TestCameraModel.cpp)
add_compile_definitions(TestCameraModel PRIVATE TEST_DATA="${TEST_DATA}")
target_link_libraries(TestCameraModel ${TEST_DEPS})
