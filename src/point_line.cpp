#include "lms/imaging_detection/point_line.h"
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
bool PointLine::findPoint(LinePoint &pointToFind,LinePoint::LinePointParam linePointParam DRAWDEBUG_PARAM){
    //find first point
    //Draw red cross
    DRAWCROSS(linePointParam.x,linePointParam.y,255,0,0);
    return  pointToFind.find(linePointParam DRAWDEBUG_ARG);
}
bool PointLine::find(const PointLineParam &param DRAWDEBUG_PARAM){
    setParam(param);
    return find(DRAWDEBUG_ARG_N);
}

void PointLine::setParam(const PointLineParam &param){
    m_pointLineParam = param;
}
bool PointLine::find(DRAWDEBUG_PARAM_N){
    LinePoint lp;
    points().clear();
    for(const LinePoint::LinePointParam lparam:m_pointLineParam.params()){
        if(findPoint(lp,lparam DRAWDEBUG_ARG)){
            points().push_back(lp);
        }
    }
    return points().size() > 0;
}

} //namepsace find
} //namespace imaging
} //namespace lms
