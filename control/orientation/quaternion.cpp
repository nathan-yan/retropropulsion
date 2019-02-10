/*
Copyright:  MIT License
Author:     Nathan Yan
Filename:   pid.cpp
Purpose:    A library for quaternion manipulation and useful quaternion        
            operations.
*/

#ifndef _QUATERNION_C
#define _QUATERNION_C

#include <math.h>

#define PI 3.1415926
#define E  2.7182818

using namespace std;

class Quaternion4{
    public:
        float a;
        float b;
        float c;
        float d;

        Quaternion4(float a, float b, float c, float d){
            this->a = a;
            this->b = b;
            this->c = c;
            this->d = d;
        }

        Quaternion4 rotate(Quaternion4 other, float angle);
        void update(float a, float b, float c, float d);
        void update(Quaternion4 other);
        
        Quaternion4 normalize();
        void normalize_();

        void multiply_(Quaternion4 other);
        void multiply_(float other); 
        Quaternion4 multiply(float other);
        void add_(Quaternion4 other);
        Quaternion4 add(Quaternion4 other);
        void add_(float other);
        void subtract_(Quaternion4 other);
        Quaternion4 subtract(Quaternion4 other);
        float magnitude();
        Quaternion4 multiply(Quaternion4 rotation);
};

Quaternion4 Quaternion4::multiply(Quaternion4 other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float r = a * other.a - b * other.b - c * other.c - d * other.d;
    float i = a * other.b + b * other.a + c * other.d - d * other.c;
    float j = a * other.c - b * other.d + c * other.a + d * other.b;
    float k = a * other.d + b * other.c - c * other.b + d * other.a;

    return Quaternion4(r, i, j, k);  
}

void Quaternion4::multiply_(Quaternion4 other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float r = a * other.a - b * other.b - c * other.c - d * other.d;
    float i = a * other.b + b * other.a + c * other.d - d * other.c;
    float j = a * other.c - b * other.d + c * other.a + d * other.b;
    float k = a * other.d + b * other.c - c * other.b + d * other.a;

    this->a = r;
    this->b = i;
    this->c = j;
    this->d = k;
}

Quaternion4 Quaternion4::multiply(float other){
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float r = a * other;
    float i = b * other;
    float j = c * other;
    float k = d * other;

    return Quaternion4(r, i, j, k);
}

void Quaternion4::subtract_(Quaternion4 other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    this->a -= other.a;
    this->b -= other.b;
    this->c -= other.c;
    this->d -= other.d;
}

Quaternion4 Quaternion4::subtract(Quaternion4 other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    return Quaternion4(a - other.a, b - other.b, c - other.c, d - other.d);
}

void Quaternion4::add_(Quaternion4 other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    this->a += other.a;
    this->b += other.b;
    this->c += other.c;
    this->d += other.d;
}

void Quaternion4::add_(float other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    this->a += other;
    this->b += other;
    this->c += other;
    this->d += other;
}

Quaternion4 Quaternion4::add(Quaternion4 other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    return Quaternion4(
        a + other.a,
        b + other.b,
        c + other.c,
        d + other.d
    );
}

void Quaternion4::multiply_(float other) {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    this->a *= other;
    this->b *= other;
    this->c *= other;
    this->d *= other;
}

float Quaternion4::magnitude(){
    return sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2) + pow(d, 2) + 0.01);
}

Quaternion4 Quaternion4::normalize() {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float length = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2) + pow(d, 2) + 0.01);

    return Quaternion4(
        a / length,
        b / length,
        c / length,
        d / length
    );
}


Quaternion4 Quaternion4::rotate(Quaternion4 rotate, float angle) {
    angle /= 2;     // Quaternion rotation doubles the angle

    float s = sin(angle * PI / 180);

    Quaternion4 q(
        cos(angle * PI / 180),
        s * rotate.b,
        s * rotate.c,
        s * rotate.d
    );

    Quaternion4 q_ (
        cos(angle * PI / 180),
        -s * rotate.b,
        -s * rotate.c,
        -s * rotate.d
    );

    return q.multiply(Quaternion4(this->a, this->b, this->c, this->d)).multiply(q_);
}

void Quaternion4::update(float a, float b, float c, float d){
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;
}

void Quaternion4::update(Quaternion4 other){
    this->a = other.a;
    this->b = other.b;
    this->c = other.c;
    this->d = other.d;
}

void Quaternion4::normalize_() {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float length = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2) + pow(d, 2) + 0.01);

    this->update(
        a / length,
        b / length,
        c / length,
        d / length
    );
}

#endif