#include "lms/imaging_detection/splitted_line.h"
namespace lms{
namespace imaging{
namespace find{
bool SplittedLine::find(SplittedLineParam &lineParam DRAWDEBUG_PARAM){
    setParam(lineParam);
    return find(DRAWDEBUG_ARG_N);
}

bool SplittedLine::find(DRAWDEBUG_PARAM_N){
    //try to find first line
    Line l;
    if(m_lines.size() > 0){
        //TODO just for verify, not that smart as verify should be changed
        l = m_lines[0];
        m_lines.clear();
    }
    Line::LineParam lineParam = m_param;
    lineParam.maxLength = m_param.lineMaxLength;
    int findCount = 0;
    float totalLength = 0;
    while(findLine(l,lineParam DRAWDEBUG_ARG)){
        m_lines.push_back(l);
        totalLength += l.length();
        //get highest point
        lms::math::vertex2f top;
        lms::math::vertex2f bot;
        float yMax = 0;
        float yMin = INFINITY;
        for(const LinePoint &lp:l.points()){
            float y = 240-lp.low_high.y;
            if(y > yMax){
                yMax = y;
                top = lp.low_high;
            }
            if(y < yMin){
                yMin = y;
                bot = lp.low_high;
            }
        }
        float angle = (top-bot).angle();
        lineParam.x = top.x +m_param.distanceBetween*cos(angle);
        //- because of coord-sys
        lineParam.y = top.y +m_param.distanceBetween*sin(angle);
        lineParam.searchAngle = angle +M_PI_2l;
        findCount++;
        //TODO in config
        if(findCount > 3){
            break;
        }
        if(totalLength > m_param.maxLength){
            break;
        }
        l.points().clear();
    }
    return m_lines.size() > 0;
}

bool SplittedLine::findLine(Line &l,Line::LineParam lineParam DRAWDEBUG_PARAM){
    float currentDistance = 0;
    bool found = true;
    while(!l.find(lineParam DRAWDEBUG_ARG)){
        currentDistance += m_param.distanceBetween/3.0;
        lineParam.y = lineParam.y -m_param.distanceBetween/3.0;
        if(currentDistance >= m_param.distanceBetween){
            found = false;
            break;
        }
    }
    return found;
}

void SplittedLine::setParam(const SplittedLineParam &lineParam){
    this->m_param = lineParam;
}

const std::vector<Line>& SplittedLine::lines() const{
    return m_lines;
}
int SplittedLine::getType() const{
    return SplittedLine::TYPE;
}
}
}
}
