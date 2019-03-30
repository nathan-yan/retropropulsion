#include <Arduino.h>
#include "../../util/utils.h"

void intersection_of_two_circles(float res[], float c1_x, float c1_y, float c1_r, float c2_x, float c2_y, float c2_r){
    float R = sqrt((c1_x - c2_x) * (c1_x - c2_x) + (c1_y - c2_y) * (c1_y - c2_y));

    // This solution is one of two, the first point is an extraneous solution
    float x2 = 0.5 * (c1_x + c2_x) + (c1_r * c1_r - c2_r * c2_r)/(2 * R * R) * (c2_x - c1_x) - 0.5 * sqrt(2 * (c1_r * c1_r + c2_r * c2_r)/(R*R) - pow((c1_r*c1_r - c2_r*c2_r), 2)/(R * R * R * R) - 1) * (c2_y - c1_y);
    float y2 = 0.5 * (c1_y + c2_y) + (c1_r * c1_r - c2_r * c2_r)/(2 * R * R) * (c2_y - c1_y) - 0.5 * sqrt(2 * (c1_r * c1_r + c2_r * c2_r)/(R*R) - pow((c1_r*c1_r - c2_r*c2_r), 2)/(R * R * R * R) - 1) * (c1_x - c2_x);

    res[0] = x2;
    res[1] = y2;
}

// All of the following terms are in centimeters
float gimbal_hole_to_pivot_distance = 4.2;
float sy_x = -2.55;
float sy_y = 2.9;

float sx_x = -2.8;
float sx_y = 2.9;

float servo_hole_distance = 1.1;
float linkage_length = 2.8;

float gimbalYawAngle(float angle){
  angle *= PI / 180;

  float px = gimbal_hole_to_pivot_distance * sin(angle);
  float py = gimbal_hole_to_pivot_distance * cos(angle);

  float res[2];
  intersection_of_two_circles(res, sx_x, sx_y, servo_hole_distance, px, py, linkage_length);

  float x = res[0] - sx_x;
  float y = res[1] - sx_y;

  /*SerialUSB.println("YAW INFO");

  SerialUSB.print(res[0]);
  SerialUSB.print(" ");
  SerialUSB.println(res[1]);

  SerialUSB.println(atan(x / y) * 180 / PI);
  SerialUSB.println("YAW");*/

  return atan(x / y) * 180 / PI;
}

float gimbalPitchAngle(float angle){
  angle *= PI / 180;

  float px = gimbal_hole_to_pivot_distance * sin(angle);
  float py = gimbal_hole_to_pivot_distance * cos(angle);

  float res[2];
  intersection_of_two_circles(res, sy_x, sy_y, servo_hole_distance, px, py, linkage_length);

  float x = res[0] - sy_x;
  float y = res[1] - sy_y;

  /*
  SerialUSB.println("PITCH INFO");

  SerialUSB.print(res[0]);
  SerialUSB.print(" ");
  SerialUSB.println(res[1]);

  SerialUSB.println(atan(x / y) * 180 / PI);
  SerialUSB.println("PITCH2.8");*/

  return atan(x / y) * 180 / PI;
}