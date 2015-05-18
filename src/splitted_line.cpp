#include "lms/imaging_detection/splitted_line.h"
namespace lms{
namespace imaging{
namespace find{
bool SplittedLine::find(SplittedLineParam &lineParam DRAWDEBUG_PARAM){
    setParam(lineParam);
    return find(DRAWDEBUG_ARG_N);
}

bool SplittedLine::find(DRAWDEBUG_PARAM_N){
    m_lines.clear();
    //try to find first line
    Line l;
    //try to find first line
    Line::LineParam lineParam = m_param;
    while(findLine(l,lineParam DRAWDEBUG_ARG)){
        m_lines.push_back(l);
        lms::math::vertex2f diff = l.points()[0].high_low-l.points()[l.points().size()-1].high_low;
        float angle = diff.angle();
        //we are looking for the angle points upwards (quite bad)
        if(angle < 0){
            angle += M_PI;
        }
        lineParam.x = m_param.distanceBetween*0.3*cos(angle);
        lineParam.x = m_param.distanceBetween*0.3*sin(angle);
        float length = l.length();
        std::cout << "length "<<l.length()<<std::endl;
        if(length > m_param.lineMaxLength || length < m_param.lineMinLength)
            break;
    }
    return m_lines.size() > 0;
}

bool SplittedLine::findLine(Line &l,Line::LineParam lineParam DRAWDEBUG_PARAM){
    float currentDistance = 0;
    bool found = true;
    while(!l.find(lineParam DRAWDEBUG_ARG)){
        currentDistance += m_param.distanceBetween/3.0;
        lineParam.y = m_param.y -currentDistance;
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
}
}
}
