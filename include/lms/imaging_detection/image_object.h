#ifndef IMAGE_OBJECT_H
#define IMAGE_OBJECT_H
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
};
}
}
}

#endif //IMAGE_OBJECT_H
