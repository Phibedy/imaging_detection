#ifndef IMAGING_DETECTION_SPLITTED_LINE_H
#define IMAGING_DETECTION_SPLITTED_LINE_H
#include "line.h"
#include <vector>
namespace lms{
namespace imaging{
namespace find{
class SplittedLine{
    struct SplittedLineParam:public Line::LineParam{
        SplittedLineParam():distanceBetween(0),lineMinLength(0),lineMaxLength(0){

        }

        float distanceBetween;
        float lineMinLength;
        float lineMaxLength;
    };

    std::vector<Line> m_lines;
    SplittedLineParam m_param;
public:
    typedef SplittedLineParam parameterType;

    bool find(SplittedLineParam &param DRAWDEBUG_PARAM);
    bool find(DRAWDEBUG_PARAM_N);
    bool findLine(Line &l,Line::LineParam lineParam DRAWDEBUG_PARAM);

    void setParam(const SplittedLineParam &param);
    const std::vector<Line>& lines() const;

};

}
}
}

#endif
