#include "lms/imaging_detection/sobel_array.h"
namespace lms{
namespace imaging{
namespace detection{


bool SobelArray::find(DRAWDEBUG_PARAM_N){
    sobelVals.clear();
    //pointer so it can be set in the bresenhamLine-function
    bool found= false;
    //end-points for the bresenham-function
    int xMax = m_searchParam.x+m_searchParam.searchLength*cos(m_searchParam.searchAngle);
    int yMax = m_searchParam.y+m_searchParam.searchLength*sin(m_searchParam.searchAngle);

    lms::math::bresenhamLine(m_searchParam.x,m_searchParam.y,xMax,yMax,[this,&found DRAWDEBUG_CAPTURE](int _x, int _y){
        //check if points are inside the image
        if(_x < 0 || _x > m_searchParam.target->width() || _y < 0 || _y >m_searchParam.target->height())
            return false;
        //draw debug point
        DRAWPOINT(_x,_y,0,0,255);
        //gauss surrounding

        int xMin = _x-2;
        int xMax = _x+2;
        int yMin = _y-2;
        int yMax = _y+2;
        /*
         * TODO that could be optimized as the next pixel will be inside the gaussed rectangle!
         * That's why we don't need to gauss a rectangle. Gaussing 3 to 5 pixel instead of 9 would ne enough
         */
        op::gaussBox(*m_searchParam.target,*m_searchParam.gaussBuffer,xMin,yMin,xMax,yMax);
        SobelVal sv;
        sv.xPos = _x;
        sv.yPos = _y;
        sv.sobelX = -op::sobelX(_x,_y,*m_searchParam.gaussBuffer);
        sv.sobelY = op::sobelY(_x,_y,*m_searchParam.gaussBuffer);
        sobelVals.push_back(sv);
        return true;
    });
    return sobelVals.size() > 0;
}

bool SobelArray::find(const SobelArrayParam &param DRAWDEBUG_PARAM){
    setParam(param);
    return find(DRAWDEBUG_ARG_N);
}
} //namepsace find
} //namespace imaging
} //namespace lms
