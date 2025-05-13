#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <Arduino.h>
#include "motor_controle.hpp"
#include "hall_sensor.hpp"
#include "param.hpp"
#include "utile.hpp"

float AngleDiffRad(float from, float to);

#define READY 0
#define IN_PROGESS 1

#define KLigne -19.0f
#define KAngle 0.1f

#define ANGLE_PRECISION 0.05 //en radian
#define DISTANCE_PRECISION 0.05f //en m defaut 0.04f

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

        void updateMotors(bool allow_backward);
        void updateMotorsRotation();

        void set_PID_L(float Kp, float Ki, float Kd);
        void set_PID_R(float Kp, float Ki, float Kd);

        void updatePID_linear_angular_mode();

        int etat = READY;

        float integral_linear = 0;
        float integral_angular = 0;

        Motor* motor_left;
        Motor* motor_right;

        double weeldistance = 0.203;
        
        float reel_linear_speed = 0;
        float reel_angular_speed = 0;

        float last_reel_linear_speed = 0;
        float last_reel_angular_speed = 0;

        float linear_speed = 0;
        float angular_speed = 0;

        float last_left_speed = 0;
        float last_right_speed = 0;

        float left_speed = 0;
        float right_speed = 0;

        float a = 0;
        float x = 0;
        float y = 0;

        float objectif_theta = 0;
        float objectif_x = 0;
        float objectif_y = 0;
        
        float KP_L = 1.4*200;
        float KI_L = 1.5*200;
        float KD_L = 0.3*200;

        float KP_R = 0.9*200;
        float KI_R = 2*200;
        float KD_R = 0.35*200;

        float new_consigne_linear = 0;
        float new_consigne_angular = 0;

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