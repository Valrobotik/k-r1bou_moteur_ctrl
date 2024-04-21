#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Arduino.h>
#include "motor_controle.hpp"
#include "hall_sensor.hpp"
#include "param.hpp"
#include "utile.hpp"

#define READY 0
#define IN_PROGESS 1

#define KLigne -19.0f
#define KAngle 0.1f


#define ANGLE_PRECISION 0.05 //en radian
#define DISTANCE_PRECISION 0.05f //en m defaut 0.04f
#define KP_R -2.0 //en radian/s des roues

#define GOTO_DELTA -0.02f
#define RECOMPUTE_BACKWARD true
#define POSITION_SHIFT 0.0f

#define GOTO_BASE_DISTANCE_THRESHOLD 0.04f //default 0.04f

#define ROTATE_TIME 1.0f

/* Speed parameters */
#define WHEEL_FORWARD_SPEED 0.25f
#define WHEEL_BACKWARD_SPEED 0.25f
#define WHEEL_TURN_SPEED_FORWARD 0.30f
#define WHEEL_TURN_SPEED_BACKWARD 0.30f
#define WHEEL_HIGHSPEED_FACTOR 2.0f


#define weeldistance 0.207f

float AngleDiffRad(float from, float to);

class Kr1bou {
    public:
        Kr1bou();
        void updateMotors(bool allow_backward);
        void updateMotorsRotation();
        void updateMotors_2(bool allow_backward);
        bool angleCorection(float objective_angle);
        void updateOdometry();
        void setObjectivexy(float x, float y);
        void setObjectivetheta(float theta);
        void resetMotorIntegrator();
        int etat;
        Motor* moteur1;
        Motor* moteur2;
        double x = 0;
        double y = 0;
        double a = 0;
    private:
        float objectif_x;
        float objectif_y;

        float initial_x;
        float initial_y;

        float objectif_theta;
        float last_left_speed;
        float last_right_speed;
        double da = 0;
        unsigned int dt1 = 0;
        unsigned int dt2 = 0;

        bool need_angle_correction;
};

struct Vec2f
{
  float x;
  float y;
};

#endif