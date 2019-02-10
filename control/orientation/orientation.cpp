#ifndef _ORIENTATION_C
#define _ORIENTATION_C

#include "quaternion.cpp"

class Orientation{
    public:
        float yaw = 0;
        float pitch = 0;
        float roll = 0;

        Quaternion4 quaternion = Quaternion4(1, 0, 0, 0);

        void updateOrientation(float gx, float gy, float gz, float ax, float ay, float az, float dt, float ALPHA);
        void updateYPR();
};

void Orientation::updateOrientation(float gx, float gy, float gz, float ax, float ay, float az, float dt, float ALPHA) {
  float elapsedTime = dt / 1000.;

  float gyroMagnitude = sqrt(gx * gx + gy * gy + gz * gz + 0.001);
  float gyroAngle = elapsedTime * gyroMagnitude;

  float sinGyroAngle = sin(gyroAngle / 2. * PI / 180.);

  Quaternion4 gyroRotation(
    cos(gyroAngle / 2. * PI / 180.),
    sinGyroAngle * gx / gyroMagnitude,
    sinGyroAngle * gy / gyroMagnitude,
    sinGyroAngle * gz / gyroMagnitude
  );

  quaternion.multiply_(gyroRotation);
  

  /*Quaternion4 gyroRotation(0, gx * PI / 180, gy * PI / 180, gz * PI / 180);
  Quaternion4 derivative(gyroRotation.multiply(quaternion));

  quaternion.update(quaternion.add(derivative.multiply(0.5 * elapsedTime)));*/

  Quaternion4 inverseOrientation(quaternion.a, -quaternion.b, -quaternion.c, -quaternion.d);

  Quaternion4 acceleration = quaternion.multiply(Quaternion4(0, ax, ay, az)).multiply(inverseOrientation);

  acceleration.normalize_();
  float adjAngle = acos(-acceleration.b);

  Quaternion4 n(
    0,
    -acceleration.c,
    acceleration.b,
    0
  );

  n.normalize_();

  adjAngle *= ALPHA;
  float sinAdjAngle = sin(adjAngle / 2.);
  Quaternion4 accelAdj(
    cos(adjAngle / 2.),
    sinAdjAngle * n.b,
    sinAdjAngle * n.c,
    sinAdjAngle * n.d
  );

  // roll += gy * elapsedTime;

  quaternion.update(accelAdj.multiply(quaternion));
  quaternion.normalize_();
}

void Orientation::updateYPR(){
  // this is the roll -> yaw -> pitch representation
  roll = atan2(2 * (quaternion.a * quaternion.d + quaternion.b * quaternion.c), 1 - 2 * (quaternion.c * quaternion.c + quaternion.d * quaternion.d)) * 180 / PI;
  //roll = 0;
  yaw = atan2(2 * (quaternion.a * quaternion.b + quaternion.c * quaternion.d), 1 - 2 * (quaternion.b * quaternion.b + quaternion.c * quaternion.c)) * 180 / PI;
  pitch = asin(2 * (quaternion.a * quaternion.c - quaternion.d * quaternion.b)) * 180 / PI;

  // for guidance we want yaw -> pitch -> roll representation

  /*float m23 = 2 * (quaternion.c * quaternion.d - quaternion.a * quaternion.b);
  float m11 = (quaternion.a * quaternion.a + quaternion.b * quaternion.b - quaternion.c * quaternion.c - quaternion.d * quaternion.d);
  float m31 = 2 * (quaternion.b * quaternion.d - quaternion.a * quaternion.c);
  float m21 = 2 * (quaternion.b * quaternion.c + quaternion.a * quaternion.d);
  float m22 = (quaternion.a * quaternion.a - quaternion.b * quaternion.b + quaternion.c * quaternion.c - quaternion.d * quaternion.d);*/

  /*float m23 = -sin(roll) * cos(yaw);
  float m11 = cos(yaw) * cos(pitch);
  float m31 = -cos(roll) * sin(yaw) * cos(pitch) + sin(roll) * sin(pitch);
  float m21 = sin(roll) * sin(yaw) * cos(pitch) + cos(roll) * sin(pitch);
  float m22 = -sin(roll) * sin(yaw) * sin(pitch) + cos(roll) * cos(pitch);

  
  yaw = atan2(-m31,  m11) * 180 / PI;
  pitch = atan2(m21, sqrt(1 - (m21 * m21)))  * 180 / PI;
  roll = atan2(-m23, m22) * 180 / PI;*/

  yaw = round(yaw * 10) / 10.;
  pitch = round(pitch * 10) / 10.;
  roll = round(roll * 10) / 10.;
}

#endif