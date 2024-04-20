#ifndef motor_control_hpp
#define motor_control_hpp

#include "hall_sensor.hpp"
#include "Arduino.h"

class Motor{
    public:

        float Kp;                   //proportionnel
        float Ki;                   //integral
        float Kd;                   //derivative

        uint8_t pwmoffset;          //offset de la pwm

        float errorSum;             //somme des erreurs
        float previousError;        //erreur precedente

        int PIN_CW;                 //pin moteur sens horraire
        int PIN_CCW;                //pin moteur sans trigo

        float SpeedConsign;        //Consigne de vitesse angulaire [rd.sec-1]
        float SpeedCurrent;        //actuel vitesse angulaire [rd.sec-1] (FeedBack)
        float previousSpeed;       //vitesse precedente
        uint64_t dt;                //delta t

        double WheelPerimeter;     //perimetre de la roue [mm]

        HallSensor* sensor;
        bool invert_sensor;

        void setSpeed(int pwmSpeed); //envoie vers la roue la comande pwm +/-[0,255]
        void updateSpeedPID(); //calcule le nouvelle increment de PID

        void setSpeedConsign(float linearSpeedConsign); //definit la consigne de vitesse de la roue en vitesse lineaire [mm.s-1]
        void setWeelSpeedConsign(float angularSpeedConsign); //definit la consigne de vitesse de la roue en vitesse angulaire [mm.s-1]

        float getFeedbackSpeed(unsigned int *dt);

        void setKpKiKd(float Kp, float Ki, float Kd);
        void stop();
        void setPwmOffset(uint8_t offset);

        void reset_integrator();

        Motor(int pin_CW, int pin_CCW, int sensor_CS, bool invert_sensor, double wheelPerimeter);
};





#endif