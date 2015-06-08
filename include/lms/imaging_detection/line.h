#ifndef IMAGE_LINE
#define IMAGE_LINE

#include <lms/imaging_detection/line_point.h>
#include <deque>
#include "lineBase.h"
#include <lms/imaging/draw_debug.h>
#include <functional>
namespace lms{
namespace imaging{
namespace find{

class Line:public LineBase{
public:
    struct LineParam:public LinePoint::LinePointParam{
        LineParam():stepLengthMin(0),stepLengthMax(0),maxLength(INFINITY),approxEdge(false),lineWidthTransMultiplier(2),validPoint(nullptr){
        }

        virtual void fromConfig(const lms::type::ModuleConfig *config){
            LinePointParam::fromConfig(config);
            stepLengthMax = config->get<float>("stepLengthMax",10);
            stepLengthMin = config->get<float>("stepLengthMin",2);
            maxLength = config->get<float>("maxLength",300);
            approxEdge = config->get<bool>("approxEdge",lfase);
            lineWidthTransMultiplier = config->get<float>("lineWidthTransMultiplier",1);
        }

        float stepLengthMin;
        float stepLengthMax;
        float maxLength;
        bool approxEdge;
        /**
         * @brief lineWidthTransMultiplier value that is multiplied with the stepLengthMax to move the last found point to find the new point
         */
        float lineWidthTransMultiplier;
        /**
         * @brief validPoint return true if the point is valid and should be added
         */
        std::function<bool(lms::imaging::find::LinePoint& DRAWDEBUG_PARAM)> validPoint;
    };
    typedef LineParam parameterType;
    //using LineBase::find;
    bool find(DRAWDEBUG_PARAM_N);


    bool find(LineParam lineParam DRAWDEBUG_PARAM);
    bool findPoint(LinePoint &pointToFind,LinePoint::LinePointParam linePointParam DRAWDEBUG_PARAM);

    void setParam(const LineParam &lineParam);
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
