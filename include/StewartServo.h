#ifndef STEWARTSERVO_H_
#define STEWARTSERVO_H_

#include <Arduino.h>
#include <Servo.h>

class StewartServo
{
private:
    Servo servo;
    const uint16_t HighestAnglePositionInUs;
    const uint16_t MiddleAnglePositionInUs;
    const uint16_t LowestAnglePositionInUs;

public:
    StewartServo(const uint8_t& pin, const uint16_t& highestAnglePositionInUs, const uint16_t& middleAnglePositionInUs,
        const uint16_t& lowestAnglePositionInUs);
    void SetServoAngle(const uint8_t& AngleInDegrees);
};

#endif//STEWARTSERVO_H_