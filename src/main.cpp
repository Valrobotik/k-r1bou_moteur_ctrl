#include <Arduino.h>
#include "motor_controle.hpp"
#include "hall_sensor.hpp"
#include "param.hpp"
#include <iostream>
#include <vector>
#include "utile.hpp"
#include "robot.hpp"

Kr1bou *robot;

u_int32_t start_time = 0;

bool open_loop = false;
bool stop = false;
bool continuous_speed_return = true;

bool ledstate = false;

#define Te 25000

float parseFirstFloatValue(String* str);
void checkSerialCommand();
void wait_until(unsigned long time);

void setup()
{
    // Initialisation de la communication SPI
    SPI.begin();

    // Initialisation du robot
    robot = new Kr1bou();

    // Initialisation de la communication série
    Serial.begin(115200);
    while (!Serial);

    // Initialisation des pins
    pinMode(M2_PIN_CW, OUTPUT);
    pinMode(M2_PIN_CCW, OUTPUT);
    pinMode(M1_PIN_CW, OUTPUT);
    pinMode(M1_PIN_CCW, OUTPUT);
    
    start_time = micros();

    robot->objectif_x = 1.5;
    robot->objectif_y = 0;

}

int test[2] = {0, 1};
int i = 0;

bool initialized = false;

void loop()
{
    if(initialized){
        robot->updateOdometry();
        if(continuous_speed_return)robot->printOdometry();
        if (stop) {
            robot->motor_right->setSpeed(0);
            robot->motor_left->setSpeed(0);
        }
        else if(!open_loop) {
            //robot->motor_right->updateSpeedPID();
            //robot->motor_left->updateSpeedPID();
            robot->updatePID_linear_angular_mode();
        }
    }
    wait_until(Te);  // permet un bouclage a frequence constante (BOZ)*/
}

unsigned long last_time = 0;
void wait_until(unsigned long time)
{
    while (micros()-last_time < time){
        checkSerialCommand();
        delay(3);
    }
    last_time = micros();
}

void checkSerialCommand(){
    while (Serial.available())
    {
        String commande = Serial.readStringUntil('F');
        String cmd_type;
        float value;
        while (commande.length() >= 1)
        {
            cmd_type = commande.substring(0,2); // 2 exclut
            commande.remove(0, 1); // 1 inclut
            value = parseFirstFloatValue(&commande);

            if (cmd_type == "LX"){
                robot->setLinearSpeed(value);
                robot->UpdateMotorSpeedConsigne();
                Serial.println("Linear speed set to : " + String(value));
                stop = false;
            }
            else if (cmd_type == "AX"){
                robot->setAngularSpeed(value);
                robot->UpdateMotorSpeedConsigne();
                stop = false;
            }
            else if (cmd_type == "PL"){
                robot->set_PID_L(value, robot->motor_left->Ki, robot->motor_left->Kd);
            }
            else if (cmd_type == "IL"){
                robot->set_PID_L(robot->motor_left->Kp, value, robot->motor_left->Kd);  
            }
            else if (cmd_type == "DL"){
                robot->set_PID_L(robot->motor_left->Kp, robot->motor_left->Ki, value);
            }
            else if (cmd_type == "PR"){
                robot->set_PID_R(value, robot->motor_right->Ki, robot->motor_right->Kd);
            }
            else if (cmd_type == "IR"){
                robot->set_PID_R(robot->motor_right->Kp, value, robot->motor_right->Kd);
            }
            else if (cmd_type == "DR"){
                robot->set_PID_R(robot->motor_right->Kp, robot->motor_right->Ki, value);
            }
            else if (cmd_type == "CL"){
                robot->motor_left->UpdateWeelPerimeter(value);
            }
            else if (cmd_type == "CR"){
                robot->motor_right->UpdateWeelPerimeter(value);
            }
            else if (cmd_type == "EX"){
                robot->setWeelDistance(value);
            }
            else if (cmd_type == "SX"){
                stop = true;
                robot->resetMotorIntegrator();
                robot->setAngularSpeed(0);
                robot->setLinearSpeed(0);
                robot->UpdateMotorSpeedConsigne();
                robot->motor_right->setSpeed(0);
                robot->motor_left->setSpeed(0);
            }
            else if (cmd_type == "VX"){
                continuous_speed_return = !continuous_speed_return;
            }
            else if (cmd_type == "VL"){
                stop = false;
                robot->motor_left->setSpeed(value);
            }
            else if (cmd_type == "VR"){
                stop = false;
                robot->motor_right->setSpeed(value);
            }
            else if (cmd_type == "WX"){
                robot->printOdometry();
            }
            else if (cmd_type == "BX"){
                // TODO Retour angle continue
            }
            else if (cmd_type == "BO"){
                open_loop = true;
            }
            else if (cmd_type == "BF"){
                open_loop = false;
            }
            else if (cmd_type == "GX"){
                // TODO Demande angle ponctuel
            }
            else if (cmd_type == "TX"){
                Serial.print("T");
                Serial.println(micros()-start_time);
            }
            else if (cmd_type == "QX"){
                Serial.println("Motor");
                initialized = true;
                stop = true;
                robot->resetMotorIntegrator();
                robot->setAngularSpeed(0);
                robot->setLinearSpeed(0);
                robot->UpdateMotorSpeedConsigne();
                robot->motor_right->setSpeed(0);
                robot->motor_left->setSpeed(0);
                robot->x = 0;
                robot->y = 0;
                robot->a = 0;
            }
            else{
                Serial.println("Commande inconnue");
            }
        }
    }
}


float parseFirstFloatValue(String* str)
{
    String value;
    for (uint16_t i = 1; i < str->length(); i++)
    {
        if(((str->charAt(i) < '0') || (str->charAt(i) > '9')) && (str->charAt(i) != '.') && (str->charAt(i) != '-')) break;
        value += str->charAt(i);
    }
    if (value.length() == 0) return 0;
    float v = value.toFloat();
    str->remove(0, value.length()+1);
    return v;
}