#ifndef LMS_IMAGING_WARP_H
#define LMS_IMAGING_WARP_H

#include <cstdint>
#include <array>
#include <fstream>

#include "lms/imaging/image.h"
#include "lms/math/vertex.h"
#include "lms/config.h"

// DIRTY CODE -> DO NOT READ

namespace lms {
namespace imaging {


// Camera to Vehicle
bool C2V(const lms::math::vertex2i*, lms::math::vertex2f*);

//Vehicle to Camera
bool V2C(const lms::math::vertex2f *, lms::math::vertex2i *);

float LP_Angle_to_rad(int8_t);
int8_t rad_to_angle_lp(float);

bool n2d(const float & xn, const float & yn, float & xdist, float & ydist);

void imageV2C(const Image &input, Image &output);
void imageD2N(const Image &input, Image &output);

struct WarpContent{
    static WarpContent instance;

    WarpContent() : initialized(false) {}

    bool initialized;

    int CALI_WIDTH;
    int CALI_HEIGHT;
    std::vector<float> d2nX;
    std::vector<float> d2nY;
    std::vector<float> world2cam;
    std::vector<float> cam2world;

    float Fx;
    float Fy;
    float Cx;
    float Cy;
    float K1;
    float K2;
    float K3;
    float K4;
    float K5;

    void fromHeader(){
#include "magic/cali_ab.h"
#include "magic/lutx.h"
#include "magic/luty.h"
#include "magic/n2d.h"

        //load from headers....
        /*
        CALI_WIDTH = 320;
        CALI_HEIGHT = 240;
        */
        //Ueye

        CALI_WIDTH = 752;
        CALI_HEIGHT = 410;

        world2cam.clear();
        for(float f:TA2B){
            world2cam.push_back(f);
        }
        cam2world.clear();
        for(float f:TB2A){
            cam2world.push_back(f);
        }
        for(int x = 0; x < CALI_WIDTH; x++){
            for(int y = 0; y< CALI_HEIGHT; y++){
                d2nX.push_back(D2NX[x][y]);
            }
        }
        for(int x = 0; x < CALI_WIDTH; x++){
            for(int y = 0; y< CALI_HEIGHT; y++){
                d2nY.push_back(D2NY[x][y]);
            }
        }
        Fx = Fx_;
        Fy = Fy_;
        Cx = Cx_;
        Cy = Cy_;
        K1 = K1_;
        K2 = K2_;
        K3 = K3_;
        K4 = K4_;
        K5 = K5_;

        //debug
        //debug();
    }

    void debug(){
        std::cout<<"col ="<< CALI_WIDTH<<"\n";
        std::cout<<"row ="<< CALI_HEIGHT<<"\n";
        std::cout<<"d2nXSize: " <<d2nX.size()<<"\n";
        std::cout<<"d2nYSize: " <<d2nY.size()<<"\n";

        lms::math::vertex2i vi;
        const lms::math::vertex2f world(0.4,0);
        lms::math::vertex2f worldCheck;
        V2C(&world,&vi);
        C2V(&vi,&worldCheck);
        std::cout<<"Warp cali check: " << world.x <<" | "<< worldCheck.x << " ; " <<world.y <<" | "<< worldCheck.y<< std::endl;

    }

    void fromConfig(const lms::Config *cali){
        CALI_WIDTH = cali->get<int>("col");
        CALI_HEIGHT = cali->get<int>("row");
        world2cam = cali->getArray<float>("world2cam");
        cam2world = cali->getArray<float>("cam2world");
        d2nX = cali->getArray<float>("d2nX");
        d2nY = cali->getArray<float>("d2nY");
        Fx = cali->get<float>("Fx");
        Fy = cali->get<float>("Fy");
        Cx = cali->get<float>("Cx");
        Cy = cali->get<float>("Cy");
        K1 = cali->get<float>("K1");
        K2 = cali->get<float>("K2");
        K3 = cali->get<float>("K3");
        K4 = cali->get<float>("K4");
        K5 = cali->get<float>("K5");


        //debug
        //debug();
        return;
    }

    void printWarp(){
        std::ofstream s("/home/phibedy/Dokumente/programmieren/c++/lms_repos/config_auto_drive/configs/camera/test");
        s<<"col="<< CALI_WIDTH<<"\n";
        s<<"row="<< CALI_HEIGHT<<"\n";
        s<<"d2nXSize=" <<d2nX.size()<<"\n";
        s<<"d2nYSize=" <<d2nY.size()<<"\n";
        s<<"d2nX=";
        for(uint i = 0; i < d2nX.size(); i++){
            s<<d2nX[i]<<",";
            if(i != 0 && i%CALI_WIDTH == 0){
                s<<"\\\n";
            }
        }
        s<<"\n";
        s<<"d2nY=";
        for(uint i = 0; i < d2nY.size(); i++){
            s<<d2nY[i]<<",";
            if(i != 0 && i%CALI_WIDTH == 0){
                s<<"\\\n";
            }
        }
        s<<"\n";
        s<<"Fx=" << Fx<<"\n"
        <<"Fy=" << Fy<<"\n"
        <<"Cx=" << Cx<<"\n"
        <<"Cy=" << Cy<<"\n"
        <<"K1=" << K1<<"\n"
        <<"K2=" << K2<<"\n"
        <<"K3=" << K3<<"\n"
        <<"K4=" << K4<<"\n"
        <<"K5="<< K5<<"\n";

        s<<("world2cam=");
        for(uint i = 0; i < world2cam.size(); i++){
            s<<world2cam[i]<<",";
        }
        s<<"\n";
        s<<("cam2world=");
        for(uint i = 0; i < cam2world.size(); i++){
            s<<cam2world[i]<<",";
        }

        s.close();
    }

};

}  // namespace imaging
}  // namespace lms

#endif /* LMS_IMAGING_WARP_H */
