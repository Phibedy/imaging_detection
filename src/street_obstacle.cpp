#include "lms/imaging_detection/street_obstacle.h"
#include "lms/imaging_detection/street_utils.h"
#include "lms/imaging/warp.h"

namespace lms {
namespace imaging {
namespace detection {

int StreetObstacle::getType() const{
    return StreetObstacle::TYPE;
}

bool StreetObstacle::find(DRAWDEBUG_PARAM_N){
    //set search param for detecting edges
    searchParam.edge = true; //just in case someone forgot it
    searchParam.preferVerify = false;
    searchParam.verify  =false;
    searchParam.fixedSearchAngle = true;

    //Trying to find white spaces
    //TODO it might fail if a searchpoint is on the egde of an obstacle
    //TODO would be much faster using binary search
    float minDistanceBetweenSearchPoints = 0.05;
    //get mid with given distance between points
    lms::math::polyLine2f newMid = searchParam.middleLine.getWithDistanceBetweenPoints(minDistanceBetweenSearchPoints);
    int numerOfSegmentsOrth = 4;
    float streetWidth = 0.4;
    std::vector<lms::math::polyLine2f> lines;
    //get searchPoints
    for(int i = 0; i < numerOfSegmentsOrth; i++){
        float p = ((float)i+1)/(numerOfSegmentsOrth+1);
        lines.push_back(newMid.moveOrthogonal(streetWidth*p*i));
    }
    //get color of all searchPoints
    for(int i = 0; i < (int)lines.size(); i++){
         lms::math::polyLine2f &line = lines[i];
        for(int p = 1; p < (int) line.points().size(); p++){
            //get the pixel in the image
            lms::math::vertex2f top = line.points()[i];
            lms::math::vertex2f bot = line.points()[i-1];
            lms::math::vertex2i topImage;
            lms::math::vertex2i botImage;
            lms::imaging::V2C(&top,&topImage);
            lms::imaging::V2C(&bot,&botImage);
            //gauss the pixel
            int colorTop = lms::imaging::op::gaussGrey(*searchParam.target,topImage.x, topImage.y);
            int colorBot = lms::imaging::op::gaussGrey(*searchParam.target,botImage.x, botImage.y);
            //draw debug point
            DRAWCROSS(botImage.x,botImage.y,0,255,0);
            int targetThres = 100; //TODO read from config
            if(colorTop-colorBot > targetThres){
                //may found an obstacle...
                Line edgeLine;
                //set search param
                searchParam.maxLength = (lines[0].points()[i]-lines[1].points()[i]).length()*2/3;//length used to search orthgonal;
                searchParam.searchLength = top.distance(bot);
                searchParam.x = botImage.x;
                searchParam.y = botImage.y;
                searchParam.searchAngle = (topImage-botImage).angle();
                if(edgeLine.find(searchParam DRAWDEBUG_ARG) && (int)edgeLine.points().size() > searchParam.minPointCount){
                    //found an edge that might be a obstacle
                    results.push_back(edgeLine);
                    return true;
                }
            }
        }
    }
    /*
    //Old code
    searchParam.edge = true; //just in case someone forgot it
    searchParam.preferVerify = false;
    searchParam.verify  =false;
    searchParam.fixedSearchAngle = true;
    streetWidth = 0.2;
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
    */

    return false;
}

bool StreetObstacle::find(StreetObstacle::StreetObstacleParam param DRAWDEBUG_PARAM){
    searchParam = param;
    return find(DRAWDEBUG_ARG_N);
}


} //namepsace find
} //namespace imaging
} //namespace lms
