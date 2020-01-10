#include "StewartServo.h"

StewartServo::StewartServo(const uint8_t& pin, const uint16_t& highestAnglePositionInUs, const uint16_t& middleAnglePositionInUs,
        const uint16_t& lowestAnglePositionInUs) : HighestAnglePositionInUs(highestAnglePositionInUs), MiddleAnglePositionInUs(middleAnglePositionInUs), 
        LowestAnglePositionInUs(lowestAnglePositionInUs)
{
    servo.attach(pin);
}

void StewartServo::SetServoAngle(const uint8_t& AngleInDegrees)
{
    servo.writeMicroseconds(map(AngleInDegrees, 0, 110, LowestAnglePositionInUs, HighestAnglePositionInUs));
}