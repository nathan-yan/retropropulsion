#ifndef _ORIENTATION_H
#define _ORIENTATION_H

#include "quaternion.cpp"

class Orientation{
    public:
        float yaw; 
        float pitch;
        float roll;

        Quaternion4 quaternion;

        Orientation();
        
        void updateOrientation(float gx, float gy, float gz, float ax, float ay, float az, float dt, float ALPHA);
        void updateYPR();
};

#endif