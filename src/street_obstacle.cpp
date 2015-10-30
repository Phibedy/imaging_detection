#include "lms/imaging_detection/street_obstacle.h"
#include "lms/imaging_detection/street_utils.h"
#include "lms/imaging/warp.h"

namespace lms {
namespace imaging {
namespace find {

int StreetObstacle::getType() const{
    return StreetObstacle::TYPE;
}

bool StreetObstacle::find(DRAWDEBUG_PARAM_N){
    //TODO we could improve performance by checkig the color-values along the path at first to check if there could be an obstacle

    searchParam.edge = true; //just in case someone forgot it
    searchParam.preferVerify = false;
    searchParam.verify  =false;
    searchParam.fixedSearchAngle = true;
    float streetWidth = 0.2;
    for(int i = 1; i < (int)searchParam.middleLine.points().size(); i++){
        lms::math::vertex2f top = searchParam.middleLine.points()[i];
        lms::math::vertex2f bot = searchParam.middleLine.points()[i-1];
        lms::math::vertex2f diff = top-bot;
        diff.rotateAntiClockwise90deg();
        diff = diff.normalize();
        diff = diff*streetWidth;
        lms::math::vertex2i boxLength1;
        lms::math::vertex2i boxLength2;
        lms::imaging::V2C(&bot,&boxLength1);
        diff = bot+diff;
        lms::imaging::V2C(&diff,&boxLength2);
        float boxLengthMax = boxLength1.distance(boxLength2);
        searchParam.maxLength = boxLengthMax;

        vecToLinePointParam(bot,top,searchParam);
        if(edgeLine.find(searchParam DRAWDEBUG_ARG) && (int)edgeLine.points().size() > searchParam.minPointCount){
            return true;
        }
    }

    return false;
}

bool StreetObstacle::find(StreetObstacle::StreetObstacleParam param DRAWDEBUG_PARAM){
    searchParam = param;
    return find(DRAWDEBUG_ARG_N);
}


} //namepsace find
} //namespace imaging
} //namespace lms
