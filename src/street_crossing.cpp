#include "lms/imaging_detection/street_crossing.h"
#include "lms/imaging_detection/line_point.h"
#include "lms/imaging_detection/line.h"
#include "lms/math/vertex.h"
#include "lms/imaging_detection/street_utils.h"

namespace lms {
namespace imaging {
namespace detection {

bool StreetCrossing::find(StreetCrossing::StreetCrossingParam param DRAWDEBUG_PARAM){
    searchParam = param;
    searchParam.fixedSearchAngle = true;
    return find(DRAWDEBUG_ARG_N);
}

bool StreetCrossing::find(DRAWDEBUG_PARAM_N){
    foundStartLine = false;
    foundCrossing = false;
    oppositeStopLineFound = false;
    using lms::math::vertex2f;
    using lms::math::vertex2i;

    float streetWidth = 0.4;//width of one
    float tangentStartLineOffset = 0.1;
    float tangentRightCrossingLineStart = 0.15;
    float tangentRightCrossingLineEnd = 0.15;
    //try to find Stop-line
    LinePoint::LinePointParam lpp;
    Line::LineParam linePar;
    linePar = searchParam;
    // ToDo find valid settings
    lpp = searchParam;

    //Not smart at all but it may work :)
    for(int i = 1; i < (int)searchParam.middleLine.points().size(); i++) {
        vertex2f top = searchParam.middleLine.points()[i];
        vertex2f bot = searchParam.middleLine.points()[i-1];
        vertex2f tangentDir = top-bot;
        tangentDir = tangentDir.normalize();
        //create hint
        vertex2f norm = tangentDir.rotateClockwise90deg();

        vertex2f targetBot = bot+norm*streetWidth/2;
        vertex2f targetTop = top+norm*streetWidth/2;

        vecToLineParam(targetBot, targetTop, top, linePar);
        if(stopLine.find(linePar DRAWDEBUG_ARG)) {
            if(stopLine.points().size() < 3)
                break;

            vertex2f foundStopLine;
            //TODO filter the average point in a better way
            //average point is the middle of the stopline
            vertex2i tmp = stopLine.getAveragePoint();
            lms::imaging::C2V(&tmp,&foundStopLine);

            //check if it's not a start line
            targetBot = foundStopLine-tangentDir*tangentStartLineOffset-norm*streetWidth;
            targetTop = foundStopLine+tangentDir*2*tangentStartLineOffset-norm*streetWidth;
            vecToLineParam(targetBot,targetTop,top,linePar);

            // search start line
            if(!leftPartStartLine.find(linePar DRAWDEBUG_ARG)){
                //it's not a Start-Line
                //get the opposite stop lane
                targetBot = foundStopLine + tangentDir*streetWidth - norm*streetWidth;
                targetTop = targetBot + tangentDir *streetWidth;
                vecToLineParam(targetBot,targetTop, top, linePar);
                oppositeStopLineFound = oppositeStopLine.find(linePar DRAWDEBUG_ARG);

                //get the crossing right lane
                targetBot = foundStopLine-tangentDir*tangentRightCrossingLineStart+norm*streetWidth;
                targetTop = foundStopLine+tangentDir*tangentRightCrossingLineEnd+norm*streetWidth;
                vecToLinePointParam(targetBot,targetTop,lpp);
                bool rightCrossingLineFound = rightCrossingLine.find(lpp DRAWDEBUG_ARG);

                if(rightCrossingLineFound){
                    //store Position
                    m_x = tmp.x;
                    m_y = tmp.y;
                    //we found the crossing <3
                    foundCrossing = true;

                    //check if the crossing is blocked
                    /*
                    targetBot = foundStopLine+tangentDir*0.5;
                    targetTop = foundStopLine+tangentDir*0.5+norm*streetWidth;
                    vecToLinePointParam(targetBot,targetTop,lpp);
                    */
                    //TODO Write street_obstacle class to find obstacles along a given path
                    targetBot = foundStopLine+tangentDir*streetWidth*1.5;
                    targetTop = targetBot + norm*streetWidth*1.5;
                    vecToLinePointParam(targetBot,targetTop,lpp);
                    Line::LineParam obstacleLineParam = searchParam;
                    obstacleLineParam.x = lpp.x;
                    obstacleLineParam.y = lpp.y;
                    obstacleLineParam.searchAngle = lpp.searchAngle;
                    obstacleLineParam.searchLength = lpp.searchLength;
                    obstacleLineParam.lineWidthMax = searchParam.boxDepthSearchLength;
                    blocked = isBlocked(obstacleLineParam DRAWDEBUG_ARG);
                    return true;
                }

            }else{
                //TODO we found a Start-line!
                tmp = stopLine.getAveragePoint()+leftPartStartLine.getAveragePoint();
                tmp /= 2;
                m_x = tmp.x;
                m_y = tmp.y;
                foundStartLine = true;
                return false;
            }
        }

    }
    return false;
}

bool StreetCrossing::isBlocked(Line::LineParam lParam DRAWDEBUG_PARAM){
    lParam.edge = true;
    lParam.preferVerify = false;
    lParam.verify  =false;
    lParam.fixedSearchAngle = true;
    lParam.validPoint = [lParam,this](lms::imaging::detection::LinePoint &lp DRAWDEBUG_PARAM)->bool{
        lms::imaging::detection::EdgePoint check = lp.low_high;
        check.searchParam().x = check.x;
        check.searchParam().y = check.y;
        check.searchParam().searchLength = searchParam.boxDepthSearchLength;
        check.searchParam().searchType = lms::imaging::detection::EdgePoint::EdgeType::HIGH_LOW;
        bool found = check.find(DRAWDEBUG_ARG_N);
        //std::cout<<"VALIDATE OBSTACLE IN CROSSING: "<<!found;

        return !found;
    };
    Line obst;
    if(obst.find(lParam DRAWDEBUG_ARG)){
        //std::cout<<"POINTS FOUND: "<<obst.points().size()<<std::endl;
        return (int)obst.points().size() >= searchParam.boxPointsNeeded;//TODO Not that smart
    }
    return false;
}


int StreetCrossing::getType() const{
    return StreetCrossing::TYPE;
}


} //namepsace find
} //namespace imaging
} //namespace lms
