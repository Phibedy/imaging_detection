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
    float streetWidth = 0.4; //TODO move to config
    //set search param for detecting edges
    searchParam.edge = true; //just in case someone forgot it
    searchParam.preferVerify = false;
    searchParam.verify  =false;
    searchParam.fixedSearchAngle = true;
    //std::function<bool(lms::imaging::detection::LinePoint& DRAWDEBUG_PARAM)>
    searchParam.validPoint = [this](lms::imaging::detection::LinePoint &lp DRAWDEBUG_PARAM)->bool{
        lms::imaging::detection::EdgePoint check = lp.low_high;
        check.searchParam() = searchParam;
        check.searchParam().x = check.x;
        check.searchParam().y = check.y;
        check.searchParam().searchLength = searchParam.boxDepthSearchLength;
        check.searchParam().searchType = lms::imaging::detection::EdgePoint::EdgeType::HIGH_LOW;
        bool found = check.find(DRAWDEBUG_ARG_N);
        return !found;
    };

    //Trying to find white spaces
    //It might fail if a searchpoint is on the egde of an obstacle (Fixed using 3 points)
    //Would be faster using binary search, not sure if it's smart
    //get mid with given distance between points
    lms::math::polyLine2f newMid = searchParam.middleLine.getWithDistanceBetweenPoints(searchParam.minDistanceBetweenSearchPoints);
    std::vector<lms::math::polyLine2f> lines;
    //get searchPoints
    for(int i = 0; i < searchParam.numerOfSegmentsOrth; i++){
        float p = ((float)i+1)/(searchParam.numerOfSegmentsOrth+1);
        if(searchParam.obstacleLeft){
            lines.push_back(newMid.moveOrthogonal(-streetWidth*p));
        }else{
            lines.push_back(newMid.moveOrthogonal(streetWidth*p));
        }
    }
    //get color of all searchPoints
    for(int i = 0; i < (int)lines.size(); i++){
         lms::math::polyLine2f &line = lines[i];
        for(int p = 1; p < (int) line.points().size()-1; p++){
            //get the pixel in the image
            lms::math::vertex2f toptop = line.points()[p+1];
            lms::math::vertex2f top = line.points()[p];
            lms::math::vertex2f bot = line.points()[p-1];
            lms::math::vertex2i topTopImage;
            lms::math::vertex2i topImage;
            lms::math::vertex2i botImage;

            if(!lms::imaging::V2C(&toptop,&topTopImage)){
                continue;
            }
            if(!lms::imaging::V2C(&top,&topImage)){
                continue;
            }
            if(!lms::imaging::V2C(&bot,&botImage)){
                continue;
            }
            //gauss the pixel
            int colorTopTop = lms::imaging::op::gaussGrey(*searchParam.target,topTopImage.x, topTopImage.y);
            int colorTop = lms::imaging::op::gaussGrey(*searchParam.target,topImage.x, topImage.y);
            int colorBot = lms::imaging::op::gaussGrey(*searchParam.target,botImage.x, botImage.y);
            //draw debug point
            DRAWCROSS(botImage.x,botImage.y,0,255,0);
            if(colorTop-colorBot > searchParam.obstaclePreSearch){
                //may found an obstacle...
                Line edgeLine;
                //set search param
                //std::cout<<topImage.x << "-"<<botImage.x <<" | "<< topImage.y<< "-" <<botImage.y<<std::endl;
                if(lines.size() > 1){
                    lms::math::vertex2i leftImage;
                    lms::math::vertex2i rightImage;
                    if(!lms::imaging::V2C(&lines[0].points()[i],&leftImage)){
                        continue;
                    }
                    if(!lms::imaging::V2C(&lines[1].points()[i],&rightImage)){
                        continue;
                    }

                    searchParam.maxLength = leftImage.distance(rightImage);//length used to search orthgonal;
                    //std::cout <<"maxLengthAAAAAAAAAA: "<<searchParam.maxLength<<std::endl;
                }else
                    searchParam.maxLength = 1;

                searchParam.searchLength = topImage.distance(botImage);
                //std::cout<<"searchParam.searchLength "<<searchParam.searchLength<<std::endl;
                searchParam.x = botImage.x;
                searchParam.y = botImage.y;
                searchParam.searchAngle = (topImage-botImage).angle();
                if(edgeLine.find(searchParam DRAWDEBUG_ARG) && (int)edgeLine.points().size() > searchParam.minPointCount){
                    //found an edge that might be a obstacle
                    results.push_back(edgeLine);
                    break; //only find one obstacle per line
                }
            }else if(colorTopTop-colorBot > searchParam.obstaclePreSearch){
                //may found an obstacle...
                Line edgeLine;
                //set search param
                //std::cout<<topImage.x << "-"<<botImage.x <<" | "<< topImage.y<< "-" <<botImage.y<<std::endl;
                if(lines.size() > 1)
                    searchParam.maxLength = 2.f/3*(lines[0].points()[i]-lines[1].points()[i]).length();//length used to search orthgonal;
                else
                    searchParam.maxLength = 1;

                searchParam.searchLength = topTopImage.distance(botImage);
                //std::cout<<"searchParam.searchLength "<<searchParam.searchLength<<std::endl;
                searchParam.x = botImage.x;
                searchParam.y = botImage.y;
                searchParam.searchAngle = (topTopImage-botImage).angle();
                //searchParam.stepLengthMin =;
                if(edgeLine.find(searchParam DRAWDEBUG_ARG) && (int)edgeLine.points().size() > searchParam.minPointCount){
                    //found an edge that might be a obstacle
                    results.push_back(edgeLine);
                    break; //only find one obstacle per line
                }
            }
        }
    }
    return results.size()>0;
}

bool StreetObstacle::find(StreetObstacle::StreetObstacleParam param DRAWDEBUG_PARAM){
    searchParam = param;
    return find(DRAWDEBUG_ARG_N);
}


} //namepsace find
} //namespace imaging
} //namespace lms
