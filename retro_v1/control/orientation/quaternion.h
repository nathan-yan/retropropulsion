#ifndef _QUATERNION_H
#define _QUATERNION_H

class Quaternion4{
    public:
        float a;
        float b;
        float c;
        float d;

        Quaternion4(float a, float b, float c, float d);

        Quaternion4 rotate(Quaternion4 other, float angle);
        void update(float a, float b, float c, float d);
        void update(Quaternion4 other);

        /* 

        Quaternion operators
        Methods with an underscore suffix are in-place. 
        Method overloads are generally for elementwise vs broadcasted multiplication

        */

        Quaternion4 normalize();
        void normalize_();

        Quaternion4 multiply(Quaternion4 rotation);     
        void multiply_(Quaternion4 other);
        void multiply_(float other);
            
        Quaternion4 add(Quaternion4 other);
        void add_(Quaternion4 other);

        Quaternion4 subtract(Quaternion4 other);
        void subtract_(Quaternion4 other);

        float magnitude();        
};

#endif