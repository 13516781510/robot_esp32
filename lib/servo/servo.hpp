#pragma once
#ifndef __MOTOR__
#define __MOTOR__
#include <Arduino.h>
class servo
{
private:
    /* data */
unsigned char pin;
unsigned char pwm_pipeline;
public:
    servo(unsigned char pin,unsigned char pwm_pipeline);
    void release();
    void pick();
};

servo::servo(unsigned char pin,unsigned char pwm_pipeline)
{
    this->pin = pin;
    this->pwm_pipeline = pwm_pipeline;
    pinMode(pin, OUTPUT);
    ledcSetup(this->pwm_pipeline,5000,8);
    ledcAttachPin(this->pin, 0);
    ledcWrite(this->pwm_pipeline, 0);
}
    void servo::release(){
        ledcWrite(this->pwm_pipeline, 0);
    }
    void servo::pick(){
        ledcWrite(this->pwm_pipeline, 128);
    }

#endif