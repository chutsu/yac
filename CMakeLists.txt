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
find_package(OpenCV REQUIRED)

include_directories(lib)
add_library(
  yac
  SHARED
  lib/AprilGrid.hpp
  lib/AprilGrid.cpp
  lib/Core.hpp
  lib/Core.cpp
  lib/CameraModel.hpp
  lib/CameraModel.cpp
)
target_link_libraries(yac Eigen3::Eigen ${OpenCV_LIBS})
