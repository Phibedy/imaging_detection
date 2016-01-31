#ifndef IMAGING_DETECTION_SOBEL_ARRAY_H
#define IMAGING_DETECTION_SOBEL_ARRAY_H

#include <cmath>
#include "lms/imaging/image_factory.h"
#include "lms/imaging/draw_debug.h"
#include "lms/imaging/image.h"
#include "lms/deprecated.h"
#include "lms/math/vertex.h"
#include "lms/config.h"
#include "lms/math/curve.h"
#include "image_object.h"
namespace lms{
namespace imaging{
namespace detection{
/**
 * @brief The SobelArray class used to get sobel-values along a given line
 */
class SobelArray:public ImageObject{
public:
    static constexpr int TYPE = -1;

    struct SobelVal{
        int xPos;
        int yPos;
        int sobelX;
        int sobelY;
    };

    struct SobelArrayParam{
        int x;
        int y;
        const Image *target;
        float searchLength;
        bool useBlackList;
        lms::math::Rect blackList;
        /**
         * @brief searchAngle in rad, don't forget that y is pointing downwards!
         */
        float searchAngle;
        Image *gaussBuffer;
        SobelArrayParam():x(0),y(0),searchLength(0),useBlackList(false){

        }

        virtual void fromConfig(const lms::Config *config){
            if(config->hasKey("searchAngle"))
                searchAngle = config->get<float>("searchAngle",0);
            if(config->hasKey("searchLength"))
                searchLength = config->get<float>("searchLength",30);
            if(config->hasKey("x"))
                x = config->get<float>("x",180);
            if(config->hasKey("y"))
                y = config->get<float>("y",120);
            if(config->hasKey("useBlackList"))
                useBlackList = config->get<bool>("useBlackList",false);
            if(config->hasKey("blackList_x")){
                blackList.x = config->get<float>("blackList_x");
                blackList.y= config->get<float>("blackList_y");
                blackList.width = config->get<float>("blackList_width");
                blackList.height = config->get<float>("blackList_height");
            }
        }

    };
    std::vector<SobelVal> sobelVals; //Not sure if it should be public
private:
    SobelArrayParam m_searchParam;

public:
    typedef SobelArrayParam parameterType;
    bool find(DRAWDEBUG_PARAM_N) override;
    bool find(const SobelArrayParam &searchParam DRAWDEBUG_PARAM);
    void setParam(const SobelArrayParam &param){
        m_searchParam = param;
    }
    int getType() const;

};

} //namepsace find
} //namespace imaging
} //namespace lms

#endif // IMAGING_DETECTION_H
