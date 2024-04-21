#include <Arduino.h>
#include "motor_controle.hpp"
#include "hall_sensor.hpp"
#include "param.hpp"
#include <iostream>
#include <vector>
#include "utile.hpp"
#include "robot.hpp"

Kr1bou* robot;

using namespace std;

unsigned int dt = 0;
u_int32_t start_time = 0;

int indexOf(String str, char c);
void printPosition();


void setup() {
    SPI.begin();

    robot = new Kr1bou();

    Serial.begin(115200);
    //Serial.println("start");
    start_time = micros();
    robot->setObjectivexy(0.0, 0.0);
    robot->setObjectivetheta(0.0);
}

double position = 0;
u_int32_t previousTime = 0;

u_int32_t coef = 0;
#define Te 50 //50ms

int etat = 0;
bool pause = true;
String str = "";
void loop() {
    if(micros() > Te*1000*coef+start_time){
        coef++;
        //robot->moteur1->setSpeed(150);
        //robot->moteur2->setSpeed(150);
        
        //robot->moteur1->updateSpeedPID();
        //robot->moteur2->updateSpeedPID();
             
        robot->updateOdometry();
        if(!pause){
            if(etat==0) robot->updateMotors(true);
            else if(etat==1) robot->updateMotorsRotation();
            else if (etat==2){
                robot->moteur1->updateSpeedPID();
                robot->moteur2->updateSpeedPID();
            }
            
            
            str = String(robot->x)+";"+String(robot->y)+";"+String(robot->a)+"R";
            Serial.println(str);
            /******only for  speed test******
            robot->moteur1->setSpeed(150);
            robot->moteur2->setSpeed(150);
            robot->moteur1->getFeedbackSpeed(&dt);
            robot->moteur2->getFeedbackSpeed(&dt);
            Serial.print(robot->moteur1->SpeedCurrent);
            Serial.print(";");
            Serial.print(robot->moteur2->SpeedCurrent);
            Serial.print(";");
            Serial.print(robot->moteur1->sensor->getAngle());
            Serial.print(";");
            Serial.println(robot->moteur2->sensor->getAngle());
            ********************************/
        }
        

        if(Serial.available()>0){
            char c = Serial.read();
            //Serial.println(c);
            if(c == 'P'){
                str = Serial.readStringUntil('R');
                pause = false;
                etat = 0;
                int i = str.indexOf(';');
                float x = str.substring(0, i).toFloat();
                float y = str.substring(i+1, str.length()-1).toFloat();
                robot->setObjectivexy(x, y); 
                robot->resetMotorIntegrator();
            }
            else if(c == 'O'){
                str = Serial.readStringUntil('R');
                int i = str.indexOf(';');
                robot->x = str.substring(0, i).toFloat();
                int j = str.indexOf(';', i+1);
                robot->y = str.substring(i+1, j).toFloat();
                robot->a = str.substring(j+1, str.length()-1).toFloat();
            }
            else if(c == 'N'){
                Serial.println("Motor");
                while (Serial.available() != 0) Serial.read();
            }
            else if(c == 'S'){
                pause = true;
                robot->moteur1->setSpeed(0);
                robot->moteur2->setSpeed(0);
                while (Serial.available() != 0) Serial.read();
            }
            else if(c == 'A'){
                pause = false;
                etat = 1;
                str = Serial.readStringUntil('R');
                float a = str.substring(0, str.length()-1).toFloat();
                robot->setObjectivetheta(a);
            }
            else if(c == 'V'){
                str = Serial.readStringUntil('R');
                pause = false;
                etat = 0;
                int i = str.indexOf(';');
                float v1 = str.substring(0, i).toFloat();
                float v2 = str.substring(i+1, str.length()-1).toFloat();
                robot->moteur1->setSpeedConsign(v1);
                robot->moteur2->setSpeedConsign(v2);
                robot->resetMotorIntegrator();
            }
        }
    }
}

int indexOf(String str, char c){
    for(unsigned int i = 0; i < str.length(); i++){
        if(str[i] == c) return i;
    }
    return -1;
}

void printPosition(){
    Serial.print(robot->x);
    Serial.print(",");
    Serial.print(robot->y);
    Serial.print(",");
    Serial.print(robot->a);
    Serial.print(",");
    Serial.print(robot->moteur1->SpeedCurrent);
    Serial.print(",");
    Serial.println(robot->moteur2->SpeedCurrent);
}