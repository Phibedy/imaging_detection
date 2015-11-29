#ifndef IMAGING_DETECTING_STREET_OBSTACLE_H
#define IMAGING_DETECTING_STREET_OBSTACLE_H

#include "image_object.h"
#include "lms/math/polyline.h"
#include "line_point.h"
#include "line.h"

namespace lms {
namespace imaging {
namespace detection {
/**
 * @brief The StreetCrossing class
 * used to detect crossing given in the CaroloCup, specific code!
 */
class StreetObstacle:public ImageObject{
public:
    struct StreetObstacleParam:public Line::LineParam{
        bool obstacleLeft; //if the obstacle is on the left of the middleLine
        //In world coordinates
        lms::math::polyLine2f middleLine;
        float targetThres; //difference between target color and black
        float minDistanceBetweenSearchPoints; //distance between two points (tangetial)
        int numerOfSegmentsOrth;
        int boxDepthSearchLength;
        /**
         * @brief minPointCount number of points that has to be found to be a valid obstacle, by default 1
         */
        int minPointCount;
        StreetObstacleParam():minPointCount(1){
        }

        virtual void fromConfig(const lms::ModuleConfig *config){
            LineParam::fromConfig(config);
            targetThres = config->get<float>("targetThres",100);
            numerOfSegmentsOrth = config->get<int>("numerOfSegmentsOrth",4);
            minDistanceBetweenSearchPoints = config->get<float>("minDistanceBetweenSearchPoints",0.2);
            minPointCount = config->get<int>("minPointCount",1);
            boxDepthSearchLength = config->get<int>("boxDepthSearchLength",10);
        }
    } searchParam;

    typedef StreetObstacleParam parameterType;
    std::vector<Line> results;

    static constexpr int TYPE = 100;
    int getType() const override;

    bool find(DRAWDEBUG_PARAM_N) override;
    bool find(StreetObstacle::StreetObstacleParam param DRAWDEBUG_PARAM);



};
}
}
}
#endif //IMAGING_DETECTING_STREET_OBSTACLE_H
