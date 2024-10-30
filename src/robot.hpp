#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Arduino.h>
#include "motor_controle.hpp"
#include "hall_sensor.hpp"
#include "param.hpp"
#include "utile.hpp"

float AngleDiffRad(float from, float to);

class Kr1bou {
    public:
        Kr1bou();
        void updateOdometry();
        void setLinearSpeed(float speed);
        void setAngularSpeed(float speed);
        void UpdateMotorSpeedConsigne();
        void resetMotorIntegrator();
        void setWeelDistance(float distance);

        void printOdometry();

        int etat;
        Motor* moteur1;
        Motor* moteur2;

        double weeldistance = 0.207;
        
        float reel_linear_speed = 0;
        float reel_angular_speed = 0;

        float linear_speed = 0;
        float angular_speed = 0;


    private:
        unsigned int dt1 = 0;
        unsigned int dt2 = 0;
        unsigned int dt = 0;
        unsigned int t = 0;
};

struct Vec2f
{
  float x;
  float y;
};

#endif