#include <lms/imaging_detection/edge_point.h>
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <lms/imaging/image_factory.h>
#include <lms/math/math.h>
#include "lms/math/curve.h"

#include <lms/imaging/draw_debug.h>
#include <iostream>

namespace lms{
namespace imaging{
namespace find{


void EdgePoint::setSearchParam(const EdgePointParam &searchParam){
    m_searchParam = searchParam;
}

bool EdgePoint::find(const EdgePointParam &searchParam DRAWDEBUG_PARAM){
    setSearchParam(searchParam);
    return find(DRAWDEBUG_ARG_N);
}

bool EdgePoint::find(DRAWDEBUG_PARAM_N){
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
        /*
         * TODO that could be optimized as the next pixel will be inside the gaussed rectangle!
         * That's why we don't need to gauss a rectangle. Gaussing 3 to 5 pixel instead of 9 would ne enough
         */
        op::gaussBox(*m_searchParam.target,*m_searchParam.gaussBuffer,_x-1,_y-1,_x+1,_y+1);
        //for 5x5 sobel
        //sobel pxl
        m_sobelX = op::sobelX(_x,_y,*m_searchParam.gaussBuffer);
        m_sobelY = op::sobelY(_x,_y,*m_searchParam.gaussBuffer);
        //check if gradient of sobel is big enough
        if(pow(m_sobelX,2)+pow(m_sobelY,2) > pow(m_searchParam.sobelThreshold,2)){
            //found an edge
            //set the type
            setType();
            if(type() == m_searchParam.searchType){
                //found an edge you were looking for :)
                //calculate the angle

                m_sobelNormal = atan2(m_sobelY,m_sobelX);
                m_sobelTangent = m_sobelNormal;
                //TODO doesnt care about rotation-count of the searchAngle % PI
                if(-M_PI_2l <m_searchParam.searchAngle && m_searchParam.searchAngle < M_PI_2l){
                    m_sobelTangent-=M_PI_2l;
                }else{
                    m_sobelTangent+=M_PI_2l;
                }
                this->x() =_x;
                this->y() =_y;
                found = true;
                //stop the bresenham
                return false;
            }
        }
        //continue searching points!
        return true;
    });
    return found;
}


EdgePoint::EdgeType EdgePoint::setType() {
    float x2 = cos(m_searchParam.searchAngle);
    float y2 = -sin(m_searchParam.searchAngle);
    float scalar = sobelX()*x2+sobelY()*y2;
    if(scalar > 0){
        m_type = EdgeType::LOW_HIGH;
    }else if(scalar < 0){
        m_type = EdgeType::HIGH_LOW;
    }else{
        m_type = EdgeType::PLANE;
    }
    return m_type;
}


int EdgePoint::sobelX(){
    return m_sobelX;
}

int EdgePoint::sobelY(){
    return m_sobelY;
}


float EdgePoint::sobelTangent(){
    return m_sobelTangent;
}

float EdgePoint::sobelNormal(){
    return m_sobelNormal;
}

EdgePoint::EdgeType EdgePoint::type(){
    return m_type;
}
} //namepsace find
} //namespace imaging
} //namespace lms
