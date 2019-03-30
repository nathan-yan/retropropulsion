#include <iostream>
#include <math.h>
#include <cassert>

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
        
        Quaternion4 normalize();
        void normalize_();

    // private:
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

Quaternion4 Quaternion4::normalize() {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float length = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2) + pow(d, 2));

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

void Quaternion4::normalize_() {
    float a = this->a, b = this->b, c = this->c, d = this->d;

    float length = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2) + pow(d, 2));

    this->update(
        a / length,
        b / length,
        c / length,
        d / length
    );
}

Quaternion4 yawPitchToQuaternion(float yaw, float pitch){
    Quaternion4 a(0, 0, 0, 1); // Straight up

    Quaternion4 yawRotate(0, 1, 0, 0);
    Quaternion4 pitchRotate(0, 0, 1, 0);

    // First rotate the rocket across the yaw axis, then transform the pitch into the new frame of reference
    // then rotate the rocket around the new pitch axis
    pitchRotate = pitchRotate.rotate(yawRotate, yaw);
    a = a.rotate(yawRotate, yaw).rotate(pitchRotate, pitch);

    return a.normalize();
}

float * quaternionToYawPitch(Quaternion4 quaternion){
    float * yawPitch = new float[2];

    // z and x
    yawPitch[1] = asin(quaternion.b / quaternion.d);
    
    // z and y
    yawPitch[0] = -asin(quaternion.c / quaternion.d);

    return yawPitch;
}

void testMultiplication(){
    cout << "Testing quaternion multiplication" << endl;

    Quaternion4 a(0, 1, 2, 3);
    Quaternion4 b(0, 2, 3, 4);

    Quaternion4 r = a.multiply(b);
    
    assert(r.a == -20);
    assert(r.b == -1);
    assert(r.c == 2); 
    assert(r.d == -1);
}

void testNormalizationInPlace(){
    cout << "Testing quaternion normalization" << endl;

    Quaternion4 a(1, 1, 2, 3);

    a.normalize_();

    cout << a.a << " " << a.b << " " << a.c << " " << endl;
}

void testRotation(){
    cout << "Testing quaternion rotation" << endl;

    Quaternion4 a(0, 1, 0, 0);
    Quaternion4 b(0, 0, 0, 1);

    Quaternion4 r = a.rotate(b, 90);

    cout << r.a << " " << r.b << " " << r.c << " " << r.d << " " << endl;
}

void testConversion(){
    cout << "Testing yaw-pitch to quaternion conversion" << endl;

    Quaternion4 r = yawPitchToQuaternion(-20, 5);
    cout << r.a << " " << r.b << " " << r.c << " " << r.d << " " << endl;

    float * yawPitch = quaternionToYawPitch(r);
    
    cout << yawPitch[0] * 180/PI << " " << yawPitch[1] * 180/PI << " ";
    
    delete yawPitch;
    yawPitch = NULL;
}

void unitTests(){
    testMultiplication();
    testNormalizationInPlace();
    testRotation();
    testConversion();
}

int main(){
    unitTests();

    return 0;
}