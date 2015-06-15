#ifndef IMAGE_POINT_LINE_H
#define IMAGE_POINT_LINE_H

#include <lms/imaging_detection/line_point.h>
#include <deque>
#include "lineBase.h"
#include <lms/imaging/draw_debug.h>
#include <functional>
namespace lms{
namespace imaging{
namespace find{

class PointLine:public LineBase{
public:
    static constexpr int TYPE = 101;
    struct PointLineParam{
        /**
         * @brief validPoint return true if the point is valid and should be added
         */
        std::function<bool(lms::imaging::find::LinePoint& DRAWDEBUG_PARAM)> validPoint;


    private:
        std::vector<LinePoint::LinePointParam> m_params;
    public:
        //TODO hacky parameter that is needed for creating the debug image... (could be improved)
        const Image *target;
        void addParam(const LinePoint::LinePointParam &param){
            target = param.target;
            if(target == nullptr){
                std::cout << "point-line: target is NULL! "<<std::endl;
            }
            m_params.push_back(param);
        }
        void clearParam() {
            m_params.clear();
        }
        const std::vector<LinePoint::LinePointParam>params()const{
            return m_params;
        }

    };
    typedef PointLineParam parameterType;
    bool find(DRAWDEBUG_PARAM_N);
    bool find(const PointLineParam &lineParam DRAWDEBUG_PARAM);
    bool findPoint(LinePoint &pointToFind,LinePoint::LinePointParam linePointParam DRAWDEBUG_PARAM);

    void setParam(const PointLineParam &param);
    int getType() const override;
protected:
    PointLineParam m_pointLineParam;
};

} //namepsace find
} //namespace imaging
} //namespace lms
#endif // IMAGE_LINE
