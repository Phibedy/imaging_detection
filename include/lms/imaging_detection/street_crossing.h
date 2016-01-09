#ifndef IMAGING_DETECTING_STREET_CROSSING_H
#define IMAGING_DETECTING_STREET_CROSSING_H

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
class StreetCrossing:public ImageObject{
    int m_x;
    int m_y;
    bool isBlocked(Line::LineParam lParam DRAWDEBUG_PARAM);

public:
    Line stopLine;
    Line leftPartStartLine;
    bool foundStartLine;
    bool foundCrossing;

    bool blocked;

    int x() const {
        return m_x;
    }
    int y() const {
        return m_y;
    }

    struct StreetCrossingParam:public Line::LineParam{
        //In world coordinates
        lms::math::polyLine2f middleLine;
    } searchParam;

    typedef StreetCrossingParam parameterType;

    static constexpr int TYPE = 101;
    int getType() const override;

    bool find(DRAWDEBUG_PARAM_N) override;
    bool find(StreetCrossing::StreetCrossingParam param DRAWDEBUG_PARAM);


};


} //namepsace find
} //namespace imaging
} //namespace lms
#endif //IMAGING_DETECTING_STREET_CROSSING_H
