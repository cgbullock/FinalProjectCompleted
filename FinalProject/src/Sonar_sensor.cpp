#include <Romi32U4.h>
#include "Sonar_sensor.h"

void SonarSensor::Init(void)
{
    pinMode(pin_TRIG,OUTPUT);
    pinMode(pin_ECHO, INPUT);   
}

float SonarSensor::PrintData(void)
{
    Serial.println(ReadData());
}

float SonarSensor::ReadData(void)
{
    //assignment 1.2
    //read out and calibrate your sonar sensor, to convert readouts to distance in [cm]
    digitalWrite(pin_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_TRIG, LOW);
    return (pulseIn(pin_ECHO, HIGH)/58);
}