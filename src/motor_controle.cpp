/**
 * @file motor_controle.cpp
 * @brief Implementation of the Motor class member functions.
 */

#include "motor_controle.hpp"

// Implementation of the Motor class member functions

/**
 * @brief Constructor for the Motor class.
 * @param pin_CW The pin number for the clockwise rotation.
 * @param pin_CCW The pin number for the counter-clockwise rotation.
 * @param sensor_CS The pin number for the chip select of hall sensor.
 */
Motor::Motor(int pin_CW, int pin_CCW, int sensor_CS, bool invert_sensor, double wheelPerimeter){
    this->sensor = new HallSensor(sensor_CS, invert_sensor);
    this->PIN_CCW = pin_CCW;
    this->PIN_CW = pin_CW;
    this->WheelPerimeter = wheelPerimeter;

    this->invert_sensor = invert_sensor;
    this->dt = 0;

    this->Kp = 0;
    this->Ki = 0;
    this->Kd = 0;
    this->pwmoffset = 0;
    this->errorSum = 0;
    this->previousError = 0;
    this->SpeedConsign = 0;
    this->SpeedCurrent = 0;
    this->previousSpeed = 0;
}

/**
 * @brief Set the power of the motor.
 * @param pwm The pulse width modulation value for the motor speed.
 */
void Motor::setSpeed(int pwm){
    if(pwm>255 || pwm < -255) return;
    if(pwm>=0){
        analogWrite(this->PIN_CW, pwm);
        analogWrite(this->PIN_CCW, 0);
    }
    else{
        analogWrite(this->PIN_CW, 0);
        analogWrite(this->PIN_CCW, -pwm);
    }
}

/**
 * @brief Set the wheel speed consign in [rd.Sec].
 * @param speed The desired wheel speed.
 */
void Motor::setWeelSpeedConsign(float speed){
    this->SpeedConsign = speed;
}

/**
 * @brief Get the wheel speed consign in linear speed [m.Sec-1].
 * 
 * @param speed The desired linear wheel speed.
*/
void Motor::setSpeedConsign(float speed){
    this->SpeedConsign = speed;
}

/**
 * @brief Get the feedback speed of the wheel.
 * @return The feedback speed of the wheel.
 */
float Motor::getFeedbackSpeed(unsigned int *dt, unsigned int *t){
    this->SpeedCurrent = this->sensor->getSpeed(dt, t)*this->WheelPerimeter/(2*PI);
    this->dt = *dt;
    this->t = *t;

    return this->SpeedCurrent;
}

/**
 * @brief Update the speed PID.  
 */
void Motor::updateSpeedPID(){
    if(this->Kp == 0 && this->Ki == 0 && this->Kd == 0) {
        Serial.print("PID not set");
        return;
    }
    unsigned int dt = this->dt;
    float speed = 0.5*this->previousSpeed+0.5*this->SpeedCurrent;  // Filtre sur la vitesse de la roue pour eviter les perturbation
    this->previousSpeed = speed;
    if (this->SpeedConsign == 0) {
        this->errorSum = 0;
        this->previousError = 0;
        this->setSpeed(0);
        return;
    }
    float error = this->SpeedConsign - speed;

    if (dt < 1000000)  this->errorSum += error*(float)dt/1000000.0;
    else this->errorSum = 0;
    float motorCommand = this->Kp*error + this->Ki*this->errorSum + this->Kd*(error-this->previousError);
    //if (motorCommand > 0) motorCommand += this->pwmoffset;
    //else if(motorCommand < 0) motorCommand -= this->pwmoffset;

    if(motorCommand > 255) {
        motorCommand = 255;
        this->errorSum -= error*dt/1000000.0;
    }
    else if(motorCommand < -255){
        motorCommand = -255;
        this->errorSum -= error*dt/1000000.0;
    }
    this->previousError = error;

    this->setSpeed((int)motorCommand);
}

/**
 * @brief Set the PID parameters.
 * @param Kp The proportional gain.
 * @param Ki The integral gain.
 * @param Kd The derivative gain.
 */
void Motor::setKpKiKd(float Kp, float Ki, float Kd){
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void Motor::setPwmOffset(uint8_t offset){
    this->pwmoffset = offset;
}

void Motor::stop(){
    this->setSpeed(0);
}

void Motor::reset_integrator(){
    this->errorSum = 0;
    this->previousError = 0;
}

void Motor::UpdateWeelPerimeter(double weelPerimeter){
    this->WheelPerimeter = weelPerimeter;
}