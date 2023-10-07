#
# The interface of pose graph by Ceres And G2O  
# Author: Kang Yifei
# Email: ccyclonel@126.com
#

cmake_minimum_required( VERSION 2.8 )
project( point_cloud_to_image )

# CMake source could not be same with cmake binary.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

#set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# # OpenMP
# FIND_PACKAGE( OpenMP QUIET)
# if(OPENMP_FOUND)
#   message("OPENMP FOUND")
#   message("OPENMP C FLAGS: ${OpenMP_C_FLAGS}")
#   message("OPENMP CXX FLAGS: ${OpenMP_CXX_FLAGS}")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()

#ceres
#find_package( Ceres REQUIRED )
#include_directories( ${CERES_INCLUDE_DIRS} )

#Opencv
if(OPENCV_PATH)
    set(OpenCV_LIBS ${OPENCV_PATH}/build/lib/libopencv_imgproc.so
                    ${OPENCV_PATH}/build/lib/libopencv_highgui.so
                    ${OPENCV_PATH}/build/lib/libopencv_core.so
                    ${OPENCV_PATH}/build/lib/libopencv_imgcodecs.so)

    set(OpenCV_INCLUDE_DIRS ${OPENCV_PATH}/include)

else()
    find_package(OpenCV 3.2.0 REQUIRED)
endif()

find_package(PCL REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS}
                     ${PCL_INCLUDE_DIRS}  )

#G2O
#find_package( G2O REQUIRED )

#Eigen
#include_directories("/usr/include/eigen3")

add_executable(${PROJECT_NAME} example.cc)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
                                      ${PCL_LIBRARIES})

