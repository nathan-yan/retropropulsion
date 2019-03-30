#include <Arduino.h>
#include "../compass_v3.h"

void circleFreedomTest(Servo yaw, Servo pitch){
    for (int degree = 0; degree < 360; degree++){
        gimbalPitchServo(sin(degree * PI / 180) * 6);
        gimbalYawServo(cos(degree * PI / 180) * 6);

        delay(10);
    }
}

void crossFreedomTest(Servo yaw, Servo pitch){
    gimbalYawServo(0);
    gimbalPitchServo(0);

    delay(1500);

    gimbalYawServo(6);
    delay(550);
    gimbalYawServo(0);
    delay(550);
    gimbalPitchServo(6);
    delay(550);
    gimbalPitchServo(0);

    delay(550);
    gimbalYawServo(-6);
    delay(550);
    gimbalYawServo(0);
    delay(550);
    gimbalPitchServo(-6);
    delay(550);
    gimbalPitchServo(0);

    delay(500);
}

void smallAngleTest(Servo yaw, Servo pitch){
    gimbalYawServo(0.5);
    delay(150);
    gimbalYawServo(0);
    delay(150);
    gimbalPitchServo(0.5);
    delay(150);
    gimbalPitchServo(0);

    delay(150);
    gimbalYawServo(-0.5);
    delay(150);
    gimbalYawServo(0);
    delay(150);
    gimbalPitchServo(-0.5);
    delay(150);
    gimbalPitchServo(0);

    delay(500);
}