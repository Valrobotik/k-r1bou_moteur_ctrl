#include "robot.hpp"

float AngleDiffRad(float from, float to){
    //Return the difference between two angles in radians
    return atan2(sin(to - from), cos(to - from));
}


Kr1bou::Kr1bou() {
    moteur1 = new Motor(M1_PIN_CW, M1_PIN_CCW, M1_SENSOR_CS, true, 0.031716652*2*PI);
    moteur1->setKpKiKd(300,600,0); //300,600,0
    moteur1->setPwmOffset(0);
    moteur1->setSpeed(0);

    moteur2 = new Motor(M2_PIN_CW, M2_PIN_CCW, M2_SENSOR_CS, false, 0.031716652*2*PI);
    moteur2->setKpKiKd(300,600,0); //300,600,0
    moteur2->setPwmOffset(0);
    moteur2->setSpeed(0);

    this->linear_speed = 0;
    this->angular_speed = 0;

    this->reel_linear_speed = 0;
    this->reel_angular_speed = 0;
}

void Kr1bou::updateOdometry() {
    unsigned int dt1, dt2, dt, t;
    this->moteur1->getFeedbackSpeed(&dt1, &t);
    this->moteur2->getFeedbackSpeed(&dt2, &t);
    double r1 = this->moteur1->SpeedCurrent;
    double r2 = this->moteur2->SpeedCurrent;
    this->dt = (dt1+dt2)/2;
    this->t = t;
    this->reel_linear_speed = (r1+r2)/2;
    this->reel_angular_speed = (r2-r1)/this->weeldistance;
}

void Kr1bou::setWeelDistance(float distance){
    this->weeldistance = distance;
}

void Kr1bou::setLinearSpeed(float speed){
    this->linear_speed = speed;
}

void Kr1bou::setAngularSpeed(float speed){
    this->angular_speed = speed;
}

void Kr1bou::UpdateMotorSpeedConsigne(){
    this->moteur1->setWeelSpeedConsign(this->linear_speed - this->angular_speed*this->weeldistance/2);
    this->moteur2->setWeelSpeedConsign(this->linear_speed + this->angular_speed*this->weeldistance/2);
}

void Kr1bou::resetMotorIntegrator(){
    this->moteur1->reset_integrator();
    this->moteur2->reset_integrator();
}

void Kr1bou::printOdometry(){
    Serial.print("L");
    Serial.print(this->reel_linear_speed);
    Serial.print("A");
    Serial.print(this->reel_angular_speed);
    Serial.print("T");
    Serial.print(this->t);
    Serial.println();
}