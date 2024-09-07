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
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)


# Build libyac
add_library(
  yac
  AprilGrid.cpp
  CalibTarget.cpp
  CameraModel.cpp
  Core.cpp
)
set(yac_DEPS  Eigen3::Eigen ${OpenCV_LIBS} apriltag::apriltag apriltags)
target_link_libraries(yac ${yac_DEPS})

# Build unit-tests
link_directories(${CMAKE_PREFIX_PATH}/lib)
add_executable(TestAprilGrid TestAprilGrid.cpp)
target_link_libraries(TestAprilGrid GTest::gtest_main ${PROJECT_NAME} ${yac_DEPS})
