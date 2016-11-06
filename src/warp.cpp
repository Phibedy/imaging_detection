#include <math.h>

#include "lms/imaging/warp.h"
#include "lms/config.h"

namespace lms {
namespace imaging {

WarpContent WarpContent::instance;

bool C2V(const lms::math::vertex2i* lp, lms::math::vertex2f* rp) {
    WarpContent& defaultContent = WarpContent::instance;

    if(! defaultContent.initialized){
        std::cout << "WarpContent::instance not initialized. Please use warp_service."
                     << std::endl;
    }
    double xtemp, ytemp;

    int x = lp->x;
    int y = lp->y;
    if (defaultContent.needsDistortion){
        //Quentin LUT: Umrechnung in ein unverzerrtes Zentralprojektionsbild.
        ///TODO: Image Size
        if (x <  0 || y<0 || x >= defaultContent.CALI_WIDTH|| y >= defaultContent.CALI_HEIGHT) {
            return false;
        }
        //Man nimmt den punkt aus der lookup-table zum entzerren
        int index = x*defaultContent.CALI_HEIGHT+y;

        xtemp = defaultContent.d2nX[index];
        ytemp = defaultContent.d2nY[index];
    }
    xtemp = x;
    ytemp = y;
   //Felix: Umrechnung des unverzerrten bilds in Auto/Straßenkoordinaten.
    //fail bei 752 * 410 sind xtemp und ytemp so klein, dass a,b,c nur von dem letzten summanden abhängen
    double a = xtemp * defaultContent.cam2world[0] + ytemp * defaultContent.cam2world[1] + defaultContent.cam2world[2];
    double b = xtemp * defaultContent.cam2world[3] + ytemp * defaultContent.cam2world[4] + defaultContent.cam2world[5];
    double c = xtemp * defaultContent.cam2world[6] + ytemp * defaultContent.cam2world[7] + defaultContent.cam2world[8];

    rp->x = (a / c);  ///To M
    rp->y = (b / c);  ///To M
    return true;
}

bool V2C(const lms::math::vertex2f* rp, lms::math::vertex2i* px) {
    WarpContent& defaultContent = WarpContent::instance;

    if(!defaultContent.initialized){
        std::cout << "WarpContent::instance not initialized. Please use warp_service."
                     << std::endl;
    }
    //Felix: Umrechnung der Autokoordinaten ins unverzerrte Bild
    float x = (float)rp->x;
    float y = (float)rp->y;
    //TODO neu eingebaute limitierung, negative x-werte ergeben (0,0)
    if(x < 0 || x > 4)
        return false;
    if(y < -3 || y > 3)
        return false;
    float a = x * defaultContent.world2cam[0] + y * defaultContent.world2cam[1] + defaultContent.world2cam[2];
    float b = x * defaultContent.world2cam[3] + y * defaultContent.world2cam[4] + defaultContent.world2cam[5];
    float c = x * defaultContent.world2cam[6] + y * defaultContent.world2cam[7] + defaultContent.world2cam[8];
    x = a / c;
    y = b / c;

    if (defaultContent.needsDistortion){
        const float xNorm = x, yNorm = y; //TODO einfach entfernt
        n2d(xNorm, yNorm, x, y);
    }
    px->x = (int16_t)x;
    px->y = (int16_t)y;

    return true;
}

float LP_Angle_to_rad(int8_t dir) {
    return (M_PI/(180.))*dir;
}

int8_t rad_to_angle_lp(float r) {
    return ((180.)/M_PI) * r;
}

bool n2d(const float & xn, const float & yn, float & xdist, float & ydist) {
    WarpContent& defaultContent = WarpContent::instance;

    if(!defaultContent.initialized){
        std::cout << "WarpContent::instance not initialized. Please use warp_service."
                     << std::endl;
    }
    if(defaultContent.needsDistortion){
        const float xnorm =(float)(xn-defaultContent.Cx)/defaultContent.Fx, ynorm = (float)(yn-defaultContent.Cy)/defaultContent.Fy;
        float r2 = xnorm*xnorm + ynorm*ynorm;
        float dist = 1 + defaultContent.K1*r2 + defaultContent.K2*r2*r2;
        float dx = 2*defaultContent.K3*xnorm*ynorm + defaultContent.K4*(r2+xnorm*xnorm);
        float dy = defaultContent.K3*(r2+ynorm*ynorm) + 2*defaultContent.K4*xnorm*ynorm;

        // Distortion
        xdist = dist*xnorm + dx;
        ydist = dist*ynorm + dy;

        // Camera position
        xdist = defaultContent.Fx*xdist + defaultContent.Cx;
        ydist = defaultContent.Fy*ydist + defaultContent.Cy;
    }else{
        xdist = xn;
        ydist = yn;
    }

    return true;
}

void imageV2C(const Image &input, Image &output) {
    /*
    if(input.width() != 320 || input.height() != 240 ||
            input.format() != Format::GREY) {
        //return false;
    }
    */

    const int textureSize = 512;
    const int planeSize = 5;
    output.resize(textureSize, textureSize, Format::GREY);

    lms::math::vertex2f in;
    lms::math::vertex2i out;
    bool success;
    std::uint8_t color;

    for(int y = 0; y < textureSize; y++) {
        for(int x = 0; x < textureSize; x++) {
            in.x = planeSize / (float)textureSize * (float) x;
            in.y = -planeSize / (float)textureSize
                    * (float)(y - (float)textureSize / 2.);

            success = V2C(&in, &out);

            if(success && out.x >= 0 && out.x < input.width()
                    && out.y >= 0 && out.y < input.height()) {
                color = *(input.data() + out.y * input.width() + out.x);
            } else {
                color = 0;// 127; //TODO warum 127?
            }
            *(output.data() + y * output.width() + x) = color;
        }
    }

    //return true;
}

void imageD2N(const Image &input, Image &output) {
    output.resize(input.width(), input.height(), Format::GREY);

    for(int y = 0; y < input.height(); y++) {
        for(int x = 0; x < input.width(); x++) {
            float xn = static_cast<float>(x);
            float yn = static_cast<float>(y);
            float xd, yd;

            auto success = n2d(xn, yn, xd, yd);
            uint8_t color = 0;
            if(success && xd >= 0 && xd < input.width()
               && yd >= 0 && yd < input.height()) {
                color = *(input.data() + static_cast<uint16_t>(yd) * input.width() + static_cast<uint16_t>(xd));
            }
            *(output.data() + y * output.width() + x) = color;
        }
    }
}

}  // namespace imaging
}  // namespace lms
