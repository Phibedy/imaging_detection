#ifndef IMAGING_DETECTION_SOBEL_ARRAY_H
#define IMAGING_DETECTION_SOBEL_ARRAY_H

#include <cmath>
#include "lms/imaging/image_factory.h"
#include "lms/imaging/draw_debug.h"
#include "lms/imaging/image.h"
#include "lms/deprecated.h"
#include "lms/math/vertex.h"
#include "lms/type/module_config.h"
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
        /**
         * @brief searchAngle in rad, don't forget that y is pointing downwards!
         */
        float searchAngle;
        Image *gaussBuffer;

        virtual void fromConfig(const lms::ModuleConfig *config){
            searchAngle = config->get<float>("searchAngle",0);
            searchLength = config->get<float>("searchLength",30);
            x = config->get<float>("x",180);
            y = config->get<float>("y",120);
        }

    };
private:
    SobelArrayParam m_searchParam;
    std::vector<SobelVal> sobelVals;

public:

    typedef SobelArrayParam parameterType;
    bool find(DRAWDEBUG_PARAM_N) override;
    bool find(const SobelArrayParam &searchParam DRAWDEBUG_PARAM);
    void setParam(const SobelArrayParam &param){
        m_searchParam = param;
    }

};

} //namepsace find
} //namespace imaging
} //namespace lms

#endif // IMAGING_DETECTION_H
