if(USE_CONAN)
message(STATUS "Using CONAN")
cmake_minimum_required(VERSION 2.8.12)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()
else()
message(STATUS "NOT Using CONAN")

endif(USE_CONAN)

set ( SOURCES
    "src/edge_point.cpp"
    "src/line.cpp"
    "src/line_point.cpp"
    "src/splitted_line.cpp"
    "src/line_base.cpp"
    "src/point_line.cpp"
    "src/street_crossing.cpp"
    "src/street_obstacle.cpp"
    "src/sobel_array.cpp"
    "src/warp.cpp"
)
set (HEADERS
    "include/lms/imaging_detection/edge_point.h"
    "include/lms/imaging_detection/line.h"
    "include/lms/imaging_detection/line_point.h"
    "include/lms/imaging_detection/image_hint.h"
    "include/lms/imaging_detection/splitted_line.h"
    "include/lms/imaging_detection/line_base.h"
    "include/lms/imaging_detection/sobel_array.h"
    "include/lms/imaging_detection/point_line.h"
    "include/lms/imaging_detection/image_object.h"
    "include/lms/imaging_detection/street_crossing.h"
    "include/lms/imaging_detection/street_obstacle.h"
    "include/lms/imaging_detection/street_utils.h"
##########magic
    "include/lms/imaging/warp.h"
    "include/lms/imaging/magic/n2d.h"
    "include/lms/imaging/magic/lutx.h"
    "include/lms/imaging/magic/luty.h"
    "include/lms/imaging/magic/cali_ab.h"
)

include_directories("include")


#set compiler flags
if((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang") OR (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"))
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wreturn-type -Wpedantic ")
endif()

add_library (lms_imaging_detection SHARED ${SOURCES} ${HEADERS})
if(USE_CONAN)
target_link_libraries (lms_imaging_detection PRIVATE ${CONAN_LIBS})
else()
target_link_libraries (lms_imaging_detection PRIVATE lmscore imaging math_lib)
endif(USE_CONAN)
