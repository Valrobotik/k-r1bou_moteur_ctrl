
// #include <Arduino.h>

// // Pins for line sensors
// const int leftSensorPin = A6;    // Left sensor
// const int middleSensorPin = A3;  // Middle sensor
// const int rightSensorPin = A2;   // Right sensor

// int leftSensor = 0;
// int middleSensor = 0;
// int rightSensor = 0;

// const int whiteLineValue = 4000; // Threshold for white line detection

// // Motor pins (L293D) and speed consign
// const int leftMotorPin1 = 2;  // Left motor direction 1
// const int leftMotorPin2 = 3;  // Left motor direction 2
// const int rightMotorPin1 = 5; // Right motor direction 1
// const int rightMotorPin2 = 4;// Right motor direction 2

// int speed = 250;

// // PID constants
// const float Kp = 1.5;
// const float Ki = 0.3;
// const float Kd = 0.1;

// const float integerEpsilon = 0.75; // Threshold for integral accumulation
// const float alpha = 80; // Gain for the speed difference between motors

// // PID variables
// static float error = 0;
// static float lastError = 0;
// static float integral = 0;
// static float derivative = 0;

// unsigned long currentTime = 0;
// unsigned long lastTime = 0;

// void goBackward(int speed){
//     analogWrite(leftMotorPin1, speed*0.8);
//     analogWrite(leftMotorPin2, 0);
//     analogWrite(rightMotorPin1, speed);
//     analogWrite(rightMotorPin2, 0);
// }

// void goForward(int speed){
//     analogWrite(leftMotorPin1, 0);
//     analogWrite(leftMotorPin2, speed*0.8);
//     analogWrite(rightMotorPin1, 0);
//     analogWrite(rightMotorPin2, speed);
//     Serial.println("Forward");
// }

// void turnRight(int speed){
//     analogWrite(leftMotorPin1, 0);
//     analogWrite(leftMotorPin2, speed*0.8*0.5);
//     analogWrite(rightMotorPin1, 0);
//     analogWrite(rightMotorPin2, speed);
// }

// void turnLeft(int speed){
//     analogWrite(leftMotorPin1, 0);
//     analogWrite(leftMotorPin2, speed*0.8);
//     analogWrite(rightMotorPin1, 0);
//     analogWrite(rightMotorPin2, speed*0.5);
// }

// void stopMotors(){
//     analogWrite(leftMotorPin1, 0);
//     analogWrite(leftMotorPin2, 0);
//     analogWrite(rightMotorPin1, 0);
//     analogWrite(rightMotorPin2, 0);
// }

// bool isNotOnLine(int leftSensor, int middleSensor, int rightSensor){
//     if((leftSensor < whiteLineValue && rightSensor < whiteLineValue)||(leftSensor > whiteLineValue && middleSensor > whiteLineValue)||(rightSensor > whiteLineValue && middleSensor > whiteLineValue)){
//         return false;
//     }
//     return true;
// }

// void followLine(int leftSensor, int middleSensor, int rightSensor, float Kp, float Ki, float Kd, float& error, 
//                 float& lastError, float& integral, float& derivative, unsigned long& lastTime) {
//     // Update current time and calculate time delta
//     currentTime = millis();
    
//     float deltaTime = (currentTime - lastTime) / 1000.0; // Convert ms to seconds

//     if (deltaTime <= 0.0) {
//         deltaTime = 0.001; // Small default delta time (1 ms)
//     }

//     // Normalize sensor readings to focus on black and white values
//     int leftWeight = -1; // Left sensor contributes negatively to error
//     int middleWeight = 0; // Middle sensor is neutral
//     int rightWeight = 1; // Right sensor contributes positively to error

//     // Weighted average to calculate error based on sensor positions
//     // Higher values correspond to the darker line (invert the readings)
//     int invertedLeft = 4095 - leftSensor;
//     int invertedMiddle = 4095 - middleSensor;
//     int invertedRight = 4095 - rightSensor;

//     float sumSensors = invertedLeft + invertedMiddle + invertedRight;
//     if (sumSensors > 0) { // Avoid division by zero
//         error = ((leftWeight * invertedLeft) + (middleWeight * invertedMiddle) + (rightWeight * invertedRight)) / sumSensors;
//     } else {
//         error = 0; // Default error if all sensors are white
//     }


//     Serial.print("Error: ");
//     Serial.println(error);

//     // Accumulate integral term, factoring in deltaTime to stabilize response
//     if (abs(error) < integerEpsilon) { // Accumulate only when error is small
//         integral += error * deltaTime;
//     }

//     // Clamp integral to prevent runaway
//     integral = constrain(integral, -20, 20); // Adjust limits as needed

//     // Reset integral if off the line
//     if (isNotOnLine(leftSensor, middleSensor, rightSensor)) {
//         integral = 0; 
//     }

//     // Serial.print("Integral: ");
//     // Serial.println(integral);

//     // Calculate derivative term, accounting for change in error over time
//     derivative = (error - lastError) / deltaTime;

//     // Serial.print("Derivative: ");
//     // Serial.println(derivative);

//     // Calculate PID output
//     float output = Kp * error + Ki * integral + Kd * derivative;

//     // Serial.print("Output: ");
//     // Serial.println(output);

//     // Update motor speeds
//     int baseSpeed = speed; // Adjust the base speed to match your robot's design
//     int leftSpeed = baseSpeed - alpha*output;
//     int rightSpeed = baseSpeed + alpha*output;

//     // Apply speed limits
//     leftSpeed = constrain(leftSpeed, 0, 255);
//     rightSpeed = constrain(rightSpeed, 0, 255);

//     // Serial.print("Left Speed: "); Serial.print(leftSpeed);
//     // Serial.print("  Right Speed: "); Serial.println(rightSpeed);


//     // Send speed to motors (assuming motor control functions exist)
//     analogWrite(leftMotorPin1, 0);
//     analogWrite(leftMotorPin2, leftSpeed);
//     analogWrite(rightMotorPin1, 0);
//     analogWrite(rightMotorPin2, rightSpeed);

//     // Update last values for the next iteration
//     lastError = error;
//     lastTime = currentTime;
// }

// void setup() {
//     // Initialize motor pins as output
//     pinMode(leftMotorPin1, OUTPUT);
//     pinMode(leftMotorPin2, OUTPUT);
//     pinMode(rightMotorPin1, OUTPUT);
//     pinMode(rightMotorPin2, OUTPUT);
    
//     Serial.begin(115200);
//     lastTime = millis(); // Initialize lastTime for the first iteration
// }

// void loop() {
//     // Read sensor values
//     leftSensor = analogRead(leftSensorPin);
//     middleSensor = analogRead(middleSensorPin);
//     rightSensor = analogRead(rightSensorPin);

//     //Serial.print("Left: "); Serial.print(leftSensor);
//     //Serial.print(" Middle: "); Serial.print(middleSensor);
//     //Serial.print(" Right: "); Serial.println(rightSensor);


//     bool droite = rightSensor > whiteLineValue;
//     bool gauche = leftSensor > whiteLineValue;
//     bool milieu = middleSensor > whiteLineValue;

//     Serial.print(gauche);
//     Serial.print(" ; ");
//     Serial.print(milieu);
//     Serial.print(" ; ");
//     Serial.println(droite);

//     speed = 230;    
//     //goForward(250);
//     if(droite && !gauche && !milieu){
//         turnRight(speed);
//     } else if(gauche && !droite && !milieu){
//         turnLeft(speed);
//     } else if(milieu && !droite && !gauche){
//         goForward(speed);
//     }
//     //Follow line
//     //followLine(leftSensor, middleSensor, rightSensor, Kp, Ki, Kd, error, lastError, integral, derivative, lastTime);

//     delay(10); // Small delay for stability
// }


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


#define Te 20000

float parseFirstFloatValue(String* str);
void checkSerialCommand();
void wait_until(unsigned long time);

void setup()
{

    delay(3000);
    // Initialisation de la communication SPI
    SPI.begin();

    // Initialisation du robot
    robot = new Kr1bou();

    // Initialisation de la communication série
    Serial.begin(115200);
    //while (!Serial);

    // Initialisation des pins
    pinMode(M2_PIN_CW, OUTPUT);
    pinMode(M2_PIN_CCW, OUTPUT);
    pinMode(M1_PIN_CW, OUTPUT);
    pinMode(M1_PIN_CCW, OUTPUT);
    start_time = micros();

    robot->setLinearSpeed(0.0);
    robot->setAngularSpeed(0.0);
    robot->UpdateMotorSpeedConsigne();

    robot->UpdateMotorSpeedConsigne();


}

double testX[4] = {.0, 0.6, 0.6, .0};
double testY[4] = {.0, .0, 0.6, 0.6};
int testN = 4;
int i = 0;

bool initialized = true;




void loop()
{
    //robot->UpdateMotorSpeedConsigne();
    //robot->updateMotors(true);
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
            //robot->new_consigne_linear = 0.35;
            //robot->new_consigne_angular = 0.0;
            //robot->updatePID_linear_angular_mode();
            robot->updateMotors(false);
            if(robot->etat == READY){
                robot->objectif_theta = atan2(testY[i]-robot->objectif_y, testX[i]-robot->objectif_x);
                robot->objectif_x = testX[i];
                robot->objectif_y = testY[i];
                robot->etat = IN_PROGESS_ANGULAR;
                Serial.print("Objectif : ");
                Serial.print(robot->objectif_x);
                Serial.print(" ; ");
                Serial.print(robot->objectif_y);
                Serial.print(" ; ");
                Serial.println(robot->objectif_theta);

                i++;
                if(i == testN)i = 0;
            }
            //robot->UpdateMotorSpeedConsigne();
        }
    }
    //robot->updateMotors(true);
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