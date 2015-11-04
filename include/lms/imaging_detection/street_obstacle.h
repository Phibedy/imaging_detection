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
        //In world coordinates
        lms::math::polyLine2f middleLine;
        /**
         * @brief minPointCount number of points that has to be found to be a valid obstacle, by default 1
         */
        int minPointCount;
        StreetObstacleParam():minPointCount(1){
        }
    } searchParam;

    typedef StreetObstacleParam parameterType;
    /**
     * @brief edge of the found obstacle
     */
    Line edgeLine;

    static constexpr int TYPE = 100;
    int getType() const override;

    bool find(DRAWDEBUG_PARAM_N) override;
    bool find(StreetObstacle::StreetObstacleParam param DRAWDEBUG_PARAM);



};
}
}
}
#endif //IMAGING_DETECTING_STREET_OBSTACLE_H
