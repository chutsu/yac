include("${CMAKE_CURRENT_LIST_DIR}/apriltag3Targets.cmake")

if (NOT MSVC) 
    include(CMakeFindDependencyMacro)
    find_dependency(Threads)
endif()

