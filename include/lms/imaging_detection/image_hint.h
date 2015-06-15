#ifndef IMAGE_HINT_H
#define IMAGE_HINT_H
#include <string>
#include "lms/imaging/draw_debug.h"
#include <vector>
namespace lms{
namespace imaging{
namespace find{

class ImageHintBase{
    //TODO das ImageObject als sharedPtr hier rein!

    bool m_processed;
protected:
    bool m_success;

public:
    virtual int getHintType() const = 0;

    virtual const lms::imaging::Image* getTarget()const = 0;
    std::string name;
    virtual ~ImageHintBase(){
    }
    virtual bool find(DRAWDEBUG_PARAM_N)=0;

    void processed(bool processed){
        m_processed = processed;
    }

    bool processed(){
        return m_processed;
    }

    bool success(){
        return m_success;
    }
};

template<class T>
class ImageHint: public ImageHintBase{
public:

    T imageObject;
    typename T::parameterType parameter;

    bool find(DRAWDEBUG_PARAM_N){
        m_success = imageObject.find(parameter DRAWDEBUG_ARG);
        return m_success;
    }


    const lms::imaging::Image* getTarget() const override{
        return parameter.target;
    }

    int getHintType() const{
        return imageObject.getType();
    }

};

class HintContainer{
public:
    std::string name;
    std::vector<ImageHintBase*> hints;

    void add(ImageHintBase *hint){
        hints.push_back(hint);
    }

    void clear(){
        //TODO wo die hints löschen? -> shared pointer!
        hints.clear();
    }

    ImageHintBase* getByName(const std::string &name) {
        for(ImageHintBase *hint : hints) {
            if(hint->name == name) {
                return hint;
            }
        }
        return nullptr;
    }

    const ImageHintBase* getByName(const std::string &name) const {
        for(ImageHintBase *hint : hints) {
            if(hint->name == name) {
                return hint;
            }
        }
        return nullptr;
    }
};
}
}
}
#endif
