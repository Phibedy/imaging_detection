#include "lms/imaging_detection/street_crossing.h"
#include "lms/imaging_detection/line_point.h"
#include "lms/imaging/warp.h"
#include "lms/math/vertex.h"

namespace lms {
namespace imaging {
namespace find {

void convertVecs(lms::math::vertex2f &start, lms::math::vertex2f &end, LinePoint::LinePointParam &lpp){

    lms::math::vertex2i targetBotImage;
    lms::math::vertex2i targetTopImage;

    lms::imaging::V2C(&start,&targetBotImage);
    lms::imaging::V2C(&end,&targetTopImage);
    //create hint
    float imageSearchDistance = (targetTopImage-targetBotImage).length();
    float searchAngle = (targetTopImage-targetBotImage).angle();
    lpp.x = targetBotImage.x;
    lpp.y = targetBotImage.y;
    lpp.searchAngle = searchAngle;
    lpp.searchLength = imageSearchDistance;
}


bool StreetCrossing::find(StreetCrossing::StreetCrossingParam param DRAWDEBUG_PARAM){
    searchParam = param;
    find(DRAWDEBUG_ARG_N);
}

bool StreetCrossing::find(DRAWDEBUG_PARAM_N){
    using lms::math::vertex2f;
    using lms::math::vertex2i;
    //try to find Stop-line
    LinePoint::LinePointParam lpp;
    LinePoint lp;

    //Not smart at all but it may work :)
    for(int i = 1; i < (int)searchParam.middleLine.points().size(); i++){

        vertex2f top = searchParam.middleLine.points()[i];
        vertex2f bot = searchParam.middleLine.points()[i-1];
        vertex2f tangentDir = top-bot;
        tangentDir.normalize();
        //create hint
        vertex2f norm;
        norm.x = -tangentDir.y;
        norm.y = tangentDir.x;
        vertex2f targetBot = bot+norm*0.4;
        vertex2f targetTop = top+norm*0.4;

        convertVecs(targetBot,targetTop,lpp);

        if(lp.find(lpp DRAWDEBUG_ARG)){
            //check if it's not a start line
            targetBot = bot-norm*0.4;
            targetTop = top-norm*0.4;

            convertVecs(targetBot,targetTop,lpp);

            LinePoint leftStartLinePoint;
            if(!leftStartLinePoint.find(lpp DRAWDEBUG_ARG)){
                //it's not a Start-Line
                LinePoint rightCrossingLine;
                LinePoint oppositeStopLine;
                //get middle of possible crossing
                targetBot = targetTop + tangentDir *0.6;
                targetTop = targetBot + tangentDir * 0.4;
                convertVecs(targetBot,targetTop,lpp);
                bool oppositeStopLineFound = oppositeStopLine.find(lpp DRAWDEBUG_ARG);

                targetTop = targetBot -norm*0.4;
                convertVecs(targetBot,targetTop,lpp);
                bool rightCrossingLineFound = rightCrossingLine.find(lpp DRAWDEBUG_ARG);

                if(oppositeStopLineFound && rightCrossingLineFound){
                    //store Position
                    m_x = leftStartLinePoint.getX();
                    m_y = leftStartLinePoint.getY();

                    //TODO check if the crossing is blocked
                    blocked = false;
                    return true;
                }

            }else{
                //TODO we found a Start-line!
                return false;
            }
        }

    }
}


int StreetCrossing::getType() const{
    return StreetCrossing::TYPE;
}


} //namepsace find
} //namespace imaging
} //namespace lms
