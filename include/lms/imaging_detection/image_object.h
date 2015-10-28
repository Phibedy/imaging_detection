#ifndef IMAGE_OBJECT_H
#define IMAGE_OBJECT_H

#include "lms/imaging/draw_debug.h"
namespace lms {
namespace imaging {
namespace find {
class ImageObject{
public:
    /**
     * @brief getType
     * @return the type of the ImageObject
     */
    virtual int getType() const = 0;

    /**
     * @brief Start searching for an edge.
     * @return true if an edge of the specified type and the minimum threshold
     * is found, otherwise false
     */
    virtual bool find(DRAWDEBUG_PARAM_N);
};
}
}
}

#endif //IMAGE_OBJECT_H
