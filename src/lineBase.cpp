#include "lms/imaging_detection/lineBase.h"
#include "lms/imaging_detection/line_point.h"
#include <lms/imaging/graphics.h>
#include <cmath>
#include <lms/imaging/draw_debug.h>
#include <algorithm>
#include <iostream>
#include "lms/math/vertex.h"
#include "lms/imaging_detection/edge_point.h"
namespace lms{
namespace imaging{
namespace find{

float LineBase::length(){
    float res = 0;
    for(int i = 0; i+1 < (int)points().size();i++){
        res += points()[i].low_high.distance(points()[i+1].low_high);
    }
    return res;
}

const std::deque<LinePoint>& LineBase::points() const {
    return m_points;
}
std::deque<LinePoint>& LineBase::points() {
    return m_points;
}
} //namepsace find
} //namespace imaging
} //namespace lms
