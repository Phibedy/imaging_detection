set ( SOURCES 
    "src/edge_point.cpp"
    "src/line.cpp"
    "src/line_point.cpp"
    "src/splitted_line.cpp"
    "src/lineBase.cpp"
    "src/point_line.cpp"
)
set (HEADERS
    "include/lms/imaging_detection/edge_point.h"
    "include/lms/imaging_detection/line.h"
    "include/lms/imaging_detection/line_point.h"
    "include/lms/imaging_detection/image_hint.h"
    "include/lms/imaging_detection/splitted_line.h"
    "include/lms/imaging_detection/lineBase.h"
    "include/lms/imaging_detection/point_line.h"
)

include_directories("include")

add_library (imaging_detection SHARED ${SOURCES} ${HEADERS})
target_link_libraries (imaging_detection PRIVATE lms imaging math_lib)
