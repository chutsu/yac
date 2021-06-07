FIND_PATH(EIGEN3_INCLUDE_DIR Eigen/Core /usr/include /usr/include/eigen3 /usr/local/include /usr/local/include/eigen3)
IF ( EIGEN3_INCLUDE_DIR )
    MESSAGE(STATUS "-- Looking for Eigen3 - found")
    SET(KDL_CFLAGS "${KDL_CFLAGS} -I${EIGEN3_INCLUDE_DIR}" CACHE INTERNAL "")
ELSE ( EIGEN3_INCLUDE_DIR )
    MESSAGE(FATAL_ERROR "-- Looking for Eigen3 - not found")
ENDIF ( EIGEN3_INCLUDE_DIR )

