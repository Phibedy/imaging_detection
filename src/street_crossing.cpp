#include "lms/imaging_detection/street_crossing.h"
#include "lms/imaging_detection/line_point.h"
#include "lms/math/vertex.h"
#include "lms/imaging_detection/street_utils.h"

namespace lms {
namespace imaging {
namespace find {

bool StreetCrossing::find(StreetCrossing::StreetCrossingParam param DRAWDEBUG_PARAM){
    searchParam = param;
    return find(DRAWDEBUG_ARG_N);
}

bool StreetCrossing::find(DRAWDEBUG_PARAM_N){
    using lms::math::vertex2f;
    using lms::math::vertex2i;

    float streetWidth = 0.4;
    float tangetSearchOffet = 0.2;
    //try to find Stop-line
    LinePoint::LinePointParam lpp;
    LinePoint lp;


    //Not smart at all but it may work :)
    for(int i = 1; i < (int)searchParam.middleLine.points().size(); i++){

        vertex2f top = searchParam.middleLine.points()[i];
        vertex2f bot = searchParam.middleLine.points()[i-1];
        vertex2f tangentDir = top-bot;
        tangentDir = tangentDir.normalize();
        //create hint
        vertex2f norm;
        norm.x = -tangentDir.y;
        norm.y = tangentDir.x;
        vertex2f targetBot = bot+norm*streetWidth/2;
        vertex2f targetTop = top+norm*streetWidth/2;

        vecToLinePointParam(targetBot,targetTop,lpp);
        if(lp.find(lpp DRAWDEBUG_ARG)){
            vertex2i tmp(lp.getX(),lp.getY());
            vertex2f foundStopLine;
            lms::imaging::C2V(&tmp,&foundStopLine);

            //check if it's not a start line
            targetBot = foundStopLine-tangentDir*tangetSearchOffet-norm*streetWidth;
            targetTop = foundStopLine+tangentDir*tangetSearchOffet-norm*streetWidth;
            vecToLinePointParam(targetBot,targetTop,lpp);
            LinePoint leftStartLinePoint;
            if(!leftStartLinePoint.find(lpp DRAWDEBUG_ARG)){
                //it's not a Start-Line

                LinePoint rightCrossingLine;
                LinePoint oppositeStopLine;
                //get the opposite stop lane
                targetBot = foundStopLine + tangentDir*streetWidth*1.5 - norm*streetWidth;
                targetTop = targetBot + tangentDir *streetWidth;
                vecToLinePointParam(targetBot,targetTop,lpp);
                bool oppositeStopLineFound = oppositeStopLine.find(lpp DRAWDEBUG_ARG);

                //get the crossing right lane
                targetBot = foundStopLine-tangentDir*tangetSearchOffet+norm*streetWidth;
                targetTop = foundStopLine+tangentDir*tangetSearchOffet+norm*streetWidth;
                vecToLinePointParam(targetBot,targetTop,lpp);
                bool rightCrossingLineFound = rightCrossingLine.find(lpp DRAWDEBUG_ARG);

                if(oppositeStopLineFound && rightCrossingLineFound){
                    //store Position
                    m_x = leftStartLinePoint.getX();
                    m_y = leftStartLinePoint.getY();

                    //TODO check if the crossing is blocked
                    blocked = false;

                    //TODO Write street_obstacle class to find obstacles along a given path
                    targetBot = foundStopLine+tangentDir*streetWidth*1.5;
                    targetTop = targetBot + norm*streetWidth*1.5;
                    vecToLinePointParam(targetBot,targetTop,lpp);

                    return true;
                }

            }else{
                //TODO we found a Start-line!
                return false;
            }
        }

    }
    return false;
}


int StreetCrossing::getType() const{
    return StreetCrossing::TYPE;
}


} //namepsace find
} //namespace imaging
} //namespace lms
