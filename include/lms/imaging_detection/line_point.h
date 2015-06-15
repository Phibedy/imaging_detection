#ifndef IMAGE_LINE_POINT
#define IMAGE_LINE_POINT

#include "lms/imaging_detection/edge_point.h"
#include <lms/imaging/draw_debug.h>

namespace lms{
namespace imaging{
namespace find{


class LinePoint: public ImageObject {

public:
    static constexpr int TYPE = 2;

    struct LinePointParam:public EdgePoint::EdgePointParam{
    LinePointParam():lineWidthMin(0),lineWidthMax(0),useSobel(false),edge(false){
    }
    virtual void fromConfig(const lms::type::ModuleConfig *config){
        EdgePointParam::fromConfig(config);
        lineWidthMax = config->get<float>("lineWidthMax",10);
        lineWidthMin = config->get<float>("lineWidthMin",1);
        edge = config->get<bool>("edge",false);
        useSobel = config->get<bool>("useSobel",false);
    }
        float lineWidthMin;
        float lineWidthMax;
        /**
         * @brief useSobel if true the sobel-angle will be used to find the second point (not recommended!)
         */
        bool useSobel;
        /**
         * @brief edge if set to true it will only search for a low_high edge!
         */
        bool edge;
    };

    typedef LinePointParam parameterType;

    void setParam(const LinePointParam &param);
    LinePointParam& param();
    const LinePointParam& param() const;

    bool find(const LinePointParam &param DRAWDEBUG_PARAM);
    bool find(DRAWDEBUG_PARAM_N);

    EdgePoint low_high, high_low;
    float getAngle();
    float getSlope();
    float distance();
    int getType() const override;


private:
    LinePointParam m_LinePointParam;
};

} //namepsace find
} //namespace imaging
} //namespace lms
#endif // IMAGE_LINE_POINT
