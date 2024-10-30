#include <Arduino.h>
#include "motor_controle.hpp"
#include "hall_sensor.hpp"
#include "param.hpp"
#include <iostream>
#include <vector>
#include "utile.hpp"
#include "robot.hpp"
#include "etat.hpp"

using namespace std;

Kr1bou *robot;

u_int32_t start_time = 0;

bool open_loop = false;
bool stop = false;
bool continuous_speed_return = true;


#define Te 20

float parseFirstFloatValue(String* str);
void checkSerialCommand();
void wait_until(unsigned long time);

void setup()
{
    SPI.begin();
    robot = new Kr1bou();
    Serial.begin(115200);
    start_time = micros();
}

void loop()
{
    robot->updateOdometry();
    
    if(continuous_speed_return) { robot->printOdometry(); }
    if (stop) {
        robot->moteur1->setSpeed(0);
        robot->moteur2->setSpeed(0);
    }
    else if(open_loop) {
        robot->moteur1->setSpeed(robot->moteur1->SpeedConsign);
        robot->moteur2->setSpeed(robot->moteur2->SpeedConsign);
    }
    else {
        robot->moteur1->updateSpeedPID();
        robot->moteur2->updateSpeedPID();
    }

    wait_until(Te);  // permet un bouclage a frequence constante (BOZ)
}

unsigned long last_time = 0;
void wait_until(unsigned long time)
{
    while (micros()-last_time < time){checkSerialCommand();}
    last_time = micros();
}

void checkSerialCommand(){
    while (Serial.available()>2)
    {
        String commande = Serial.readStringUntil('\n');
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
                stop = false;
            }
            else if (cmd_type == "AX"){
                robot->setAngularSpeed(value);
                robot->UpdateMotorSpeedConsigne();
                stop = false;
            }
            else if (cmd_type == "PL"){
                robot->moteur1->setKpKiKd(value, robot->moteur1->Ki, robot->moteur1->Kd);
            }
            else if (cmd_type == "IL"){
                robot->moteur1->setKpKiKd(robot->moteur1->Kp, value, robot->moteur1->Kd);
            }
            else if (cmd_type == "DL"){
                robot->moteur1->setKpKiKd(robot->moteur1->Kp, robot->moteur1->Ki, value);
            }
            else if (cmd_type == "PR"){
                robot->moteur2->setKpKiKd(value, robot->moteur2->Ki, robot->moteur2->Kd);
            }
            else if (cmd_type == "IR"){
                robot->moteur2->setKpKiKd(robot->moteur2->Kp, value, robot->moteur2->Kd);
            }
            else if (cmd_type == "DR"){
                robot->moteur2->setKpKiKd(robot->moteur2->Kp, robot->moteur2->Ki, value);
            }
            else if (cmd_type == "CL"){
                robot->moteur1->UpdateWeelPerimeter(value);
            }
            else if (cmd_type == "CR"){
                robot->moteur2->UpdateWeelPerimeter(value);
            }
            else if (cmd_type == "EX"){
                robot->setWeelDistance(value);
            }
            else if (cmd_type == "SX"){
                stop = true;
                robot->resetMotorIntegrator();
                robot->setAngularSpeed(0);
                robot->setLinearSpeed(0);
                robot->moteur1->setSpeed(0);
                robot->moteur2->setSpeed(0);
            }
            else if (cmd_type == "VX"){
                continuous_speed_return = !continuous_speed_return;
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
    for (int i = 1; i < str->length(); i++)
    {
        if(((str->charAt(i) < '0') || (str->charAt(i) > '9')) && (str->charAt(i) != '.')) break;
        value += str->charAt(i);
    }
    if (value.length() == 0) return 0;
    float v = value.toFloat();
    str->remove(0, value.length()+1);
    return v;
}