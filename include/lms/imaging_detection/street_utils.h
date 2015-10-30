#ifndef IMAGING_DETECTION_STREET_UTILS_H
#define IMAGING_DETECTION_STREET_UTILS_H

#include "line_point.h"
#include "lms/imaging/warp.h"
namespace lms {
namespace imaging {
namespace find {
/**
 * @brief vecToLinePointParam
 * @param start
 * @param end
 * @param lpp sets the searchANgle and the searchLength of the given LinePointParam
 */
inline void vecToLinePointParam(const lms::math::vertex2f &start,const lms::math::vertex2f &end, LinePoint::LinePointParam &lpp){

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

}
}
}

#endif //IMAGING_DETECTION_STREET_UTILS_H
