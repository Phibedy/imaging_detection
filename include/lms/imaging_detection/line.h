#ifndef IMAGE_LINE
#define IMAGE_LINE

#include <lms/imaging_detection/line_point.h>
#include <deque>
#include "line_base.h"
#include <lms/imaging/draw_debug.h>
#include <functional>
namespace lms{
namespace imaging{
namespace detection{

class Line:public LineBase{
public:
    static constexpr int TYPE = 100;
    struct LineParam:public LinePoint::LinePointParam{
        LineParam():stepLengthMin(0),stepLengthMax(0),maxLength(INFINITY),approxEdge(false),lineWidthTransMultiplier(2),validPoint(nullptr){
        }

        virtual void fromConfig(const lms::ModuleConfig *config){
            LinePointParam::fromConfig(config);
            if(config->hasKey("stepLengthMax"))
                stepLengthMax = config->get<float>("stepLengthMax",10);
            if(config->hasKey("stepLengthMin"))
                stepLengthMin = config->get<float>("stepLengthMin",2);
            if(config->hasKey("maxLength"))
                maxLength = config->get<float>("maxLength",300);
            if(config->hasKey("approxEdge"))
                approxEdge = config->get<bool>("approxEdge",false);
            if(config->hasKey("lineWidthTransMultiplier"))
                lineWidthTransMultiplier = config->get<float>("lineWidthTransMultiplier",1);
            if(config->hasKey("fixedSearchAngle"))
                fixedSearchAngle = config->get<bool>("fixedSearchAngle",false);
        }
        bool fixedSearchAngle;
        float stepLengthMin;
        float stepLengthMax;
        /**
         * @brief maxLength length in one extend-direction
         */
        float maxLength;
        bool approxEdge;
        /**
         * @brief lineWidthTransMultiplier value that is multiplied with the stepLengthMax to move the last found point to find the new point
         */
        float lineWidthTransMultiplier;
        /**
         * @brief validPoint return true if the point is valid and should be added
         */
        std::function<bool(lms::imaging::detection::LinePoint& DRAWDEBUG_PARAM)> validPoint;
    };
    typedef LineParam parameterType;
    //using LineBase::find;
    bool find(DRAWDEBUG_PARAM_N);


    bool find(LineParam lineParam DRAWDEBUG_PARAM);
    bool findPoint(LinePoint &pointToFind,LinePoint::LinePointParam linePointParam DRAWDEBUG_PARAM);

    void setParam(const LineParam &lineParam);
    int getType() const override;

    lms::math::vertex2i getAveragePoint() const;
protected:

    void extend(bool direction DRAWDEBUG);
    bool verifyPoint(LinePoint &lp, LinePoint::LinePointParam lParam DRAWDEBUG_PARAM);
    //get a good point that could be found next time again with searchAngle
    bool getSearchPoint(int &x, int &y, float &angle);
    LineParam m_LineParam;
};

} //namepsace find
} //namespace imaging
} //namespace lms
#endif // IMAGE_LINE
