#include "robot.hpp"

float AngleDiffRad(float from, float to){
    //Return the difference between two angles in radians
    return atan2(sin(to - from), cos(to - from));
}

Kr1bou::Kr1bou() {
    motor_right = new Motor(M1_PIN_CW, M1_PIN_CCW, M1_SENSOR_CS, true, 0.02866*2*PI, 1.0);
    motor_right->setKpKiKd(350,600,0); //300,600,0
    motor_right->setPwmOffset(50);
    motor_right->setSpeed(0);

    motor_left = new Motor(M2_PIN_CW, M2_PIN_CCW, M2_SENSOR_CS, false, 0.02866*2*PI, 1.0);
    motor_left->setKpKiKd(280,450,0); //300,600,0
    motor_left->setPwmOffset(50);
    motor_left->setSpeed(0);

    this->linear_speed = 0;
    this->angular_speed = 0;

    this->reel_linear_speed = 0;
    this->reel_angular_speed = 0;
}

void Kr1bou::updateOdometry() {
    unsigned int dt1, dt2, t;
    this->motor_right->getFeedbackSpeed(&dt1, &t);
    this->motor_left->getFeedbackSpeed(&dt2, &t);

    double left_speed = this->motor_left->SpeedCurrent;
    double right_speed = this->motor_right->SpeedCurrent;

    this->dt = (dt1+dt2)/2;
    this->t = t;
    this->reel_linear_speed = (left_speed + right_speed)/2;
    this->reel_angular_speed = (right_speed-left_speed)/this->weeldistance;
    
    // integration selon la methode des trapezes
    float angular_speed = (0.5*this->reel_angular_speed+0.5*this->last_reel_angular_speed);
    float linear_speed = (0.5*this->reel_linear_speed+0.5*this->last_reel_linear_speed);
    if (abs(angular_speed) > 0.0001){ // dans le cas ou la vitesse angulaire est non nulle on utilise la methode de l'arc de cercle
        //methode de l'arc de cercle pour calculer la nouvelle position
        this->a = this->a + angular_speed*this->dt/1000000.0;
        float R = linear_speed/angular_speed;
        this->x = this->x - R*sin(this->a) + R*sin(this->a+angular_speed*this->dt/1000000.0);
        this->y = this->y + R*cos(this->a) - R*cos(this->a+angular_speed*this->dt/1000000.0);
    }else{
        //methode de la droite pour calculer la nouvelle position
        this->a = this->a + angular_speed*this->dt/1000000.0;
        this->x = this->x + linear_speed*cos(this->a)*this->dt/1000000.0;
        this->y = this->y + linear_speed*sin(this->a)*this->dt/1000000.0;
    }
    this->reel_angular_speed = angular_speed;
    this->reel_linear_speed = linear_speed;
    this->last_reel_angular_speed = this->reel_angular_speed;
    this->last_reel_linear_speed = this->reel_linear_speed;

    if (this->a >= 2*PI) this->a -= 2*PI;
    else if (this->a < 0) this->a += 2*PI;
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
    this->motor_right->setWeelSpeedConsign(this->linear_speed + this->angular_speed*this->weeldistance/2);
    this->motor_left->setWeelSpeedConsign(this->linear_speed - this->angular_speed*this->weeldistance/2);
}

void Kr1bou::resetMotorIntegrator(){
    this->integral_angular = 0;
    this->integral_linear = 0;
    this->motor_right->reset_integrator();
    this->motor_left->reset_integrator();
}

void Kr1bou::printOdometry(){
    Serial.print(this->x, 4);
    Serial.print(";");
    Serial.print(this->y, 4);
    Serial.print(";");
    Serial.print(this->a, 4);
    Serial.print(";");
    Serial.print(this->reel_linear_speed, 4);
    Serial.print(";");
    Serial.print(this->reel_angular_speed, 4);
    Serial.print(";");
    Serial.println(micros());
}

void Kr1bou::updateMotors(bool allow_backward) {
    //objectif à atteindre

    if (this->etat == IN_PROGESS_ANGULAR){
        this->updateMotorsRotation();
        return;
    }

    this->etat = IN_PROGESS;
    float x2 = this->objectif_x;
    float y2 = this->objectif_y;

    //float destination_angle = atan2f(y2 - this->y, x2 - this->x);

    float destination_angle = this->objectif_theta;
    
    //on shift la position pour ajusté le centre de rotation et le centre de detection
    Vec2f pos = Vec2f{(float)this->x, (float)this->y};
    pos.x -= cosf(this->a) * POSITION_SHIFT;
    pos.y -= sinf(this->a) * POSITION_SHIFT;

        /* A GOTO order is defined by a target point (center of a cell) plus a given direction */
    Vec2f p1 = Vec2f{x2, y2};
    Vec2f p2 = Vec2f{x2+ cosf(destination_angle), y2 + sinf(destination_angle)};
    /* We compute the orthogonal distance between the robot and the target line */
    float dist_to_line = abs((p2.x - p1.x) * (p1.y - pos.y) - (p1.x - pos.x) * (p2.y - p1.y)) / sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));

    /* Checking on which side of the target line we are */
    float angle_line_to_robot = atan2f(pos.y - p1.y, pos.x - p1.x);
    float diff_angle_line_to_robot = AngleDiffRad(destination_angle, angle_line_to_robot);
    bool is_on_right_side = diff_angle_line_to_robot > 0.0f;
    float target_angle = destination_angle + atan(dist_to_line * 14.0f) * (is_on_right_side ? -1.0f : 1.0f); //7.0f

    float dx_base = x2 - pos.x;
    float dy_base = y2 - pos.y;
    //float dx_target = x2 + GOTO_DELTA * cos(destination_angle) - pos.x;
    //float dy_target = y2 + GOTO_DELTA * sin(destination_angle) - pos.y;

    float dist_to_base = sqrt(dx_base * dx_base + dy_base * dy_base);

    //float angle_to_target = atan2f(dy_target, dx_target);
   //float diff_angle_direction = AngleDiffRad(angle_to_target,destination_angle);


    //bool goto_over = abs(diff_angle_direction) > M_PI / 2;


    /* When we reach the cell center (distance based, this time), we tell the Strategy that
    * we can start computing the next step, even if we still move a little bit meanwhile */
    this->etat = IN_PROGESS; 
    bool m_goto_base_reached = false;
    if (dist_to_base < GOTO_BASE_DISTANCE_THRESHOLD)
    {
        m_goto_base_reached = true;
        this->etat = READY;
    }

    /* Checking if we should go backwards instead of forward */
    float angle_diff = AngleDiffRad(target_angle, this->a);
    if (abs(angle_diff) > M_PI)
    {
        angle_diff = AngleDiffRad(target_angle, this->a + M_PI);
    }

    bool backward = abs(angle_diff) > M_PI / 2;
    //bool backward = false;


    backward = backward && allow_backward;
    if (backward)
    {
        /* If going backward, we target the opposite angle */
        angle_diff = AngleDiffRad(target_angle + M_PI, this->a);
    }

    // if (abs(angle_diff) > M_PI / 4)
    // {
    //     this->new_consigne_angular = 2.0f;
    //     this->new_consigne_angular *= angle_diff < 0 ? 1.0f : -1.0f; // Turn left or right
    //     this->new_consigne_linear = 0;
    //     this->updatePID_linear_angular_mode();
    //     return;
    // }

    /* Computing the two speed components */
    float forward_speed = 0.0f;
    float turn_speed = 0.0f;
    float speed_limit = (backward ? WHEEL_BACKWARD_SPEED : WHEEL_FORWARD_SPEED);

    ////////////////////////////////// SI High Speed multiplié ici la speed limit par WHEEL_HIGHSPEED_FACTOR 

    forward_speed = speed_limit * (1.0f - abs(angle_diff) / (M_PI / 2));

    if (backward)
    {
        forward_speed = -forward_speed;
    }

    /* Turn speed is proportional to the angle difference (just the opposite of the forward speed!) */
    turn_speed = abs(angle_diff) / (M_PI / 2);
    turn_speed *= (backward ? WHEEL_TURN_SPEED_BACKWARD : WHEEL_TURN_SPEED_FORWARD);
    turn_speed *= angle_diff < 0 ? 1.0f : -1.0f; // Turn left or right

    /* If the robot speed limit is the limiting factor instead of our parameters (because of a slow
    * down), we reduce everything proportionally so that we keep the same turn/forward ratio. */
    float reduction_factor = 1.0f;
    if (dist_to_base<0.1f)reduction_factor = dist_to_base / 0.1f;

    float left_speed = reduction_factor * (forward_speed + turn_speed);
    float right_speed = reduction_factor * (forward_speed - turn_speed);

    /////////////////////
    //goto_over = true;
    /////////////////////
    
    /* Note: we reset the PIDs if the GOTO is over, but we don't know if that's really needed */
    if (this->etat == READY)
    {
        turn_speed = 0.0f;
        forward_speed = 0.0f;
        this->motor_left->setSpeed(0);
        this->motor_right->setSpeed(0);
        return;
    }
    this->new_consigne_angular = turn_speed;
    this->new_consigne_linear = forward_speed;
    this->updatePID_linear_angular_mode();

}

void Kr1bou::updateMotorsRotation(){
    float angle = AngleDiffRad(this->a, this->objectif_theta);
    if (abs(angle) > ANGLE_PRECISION){
        float w = angle;
        if (abs(w) > 1.5)w = 1.5 * (w > 0 ? 1 : -1);
        this->new_consigne_angular = w;
        this->new_consigne_linear = 0;
        this->updatePID_linear_angular_mode();
        return;
    }
    this->resetMotorIntegrator();
    this->new_consigne_linear = 0;
    this->new_consigne_angular = 0;
    this->updatePID_linear_angular_mode();
    this->etat = IN_PROGESS;
}


void Kr1bou::updatePID_linear_angular_mode(){
    this->new_consigne_linear = this->new_consigne_linear;
    this->new_consigne_angular = this->new_consigne_angular;

    float diff_linear = this->new_consigne_linear - this->reel_linear_speed;
    float diff_angular = this->new_consigne_angular - this->reel_angular_speed;

    this->integral_linear += diff_linear*this->dt/1000000.0;
    this->integral_angular += diff_angular*this->dt/1000000.0;

    float derivative_linear = (diff_linear - (this->new_consigne_linear - this->last_reel_linear_speed))/(this->dt/1000000.0);
    float derivative_angular = (diff_angular - (this->new_consigne_angular - this->last_reel_angular_speed))/(this->dt/1000000.0);

    float linear_speed = diff_linear*this->KP_L + this->integral_linear*this->KI_L+ derivative_linear*this->KD_L;
    float angular_speed = diff_angular*this->KP_R + this->integral_angular*this->KI_R+ derivative_angular*this->KD_R;

    if (linear_speed > 250){
        linear_speed = 250;
        this->integral_linear -= diff_linear*this->dt/1000000.0;
    }else if (linear_speed < -250){
        linear_speed = -250;
        this->integral_linear -= diff_linear*this->dt/1000000.0;
    }

    if (angular_speed > 250){
        angular_speed = 250;
        this->integral_angular -= diff_angular*this->dt/1000000.0;
    }
    else if (angular_speed < -250){
        angular_speed = -250;
        this->integral_angular -= diff_angular*this->dt/1000000.0;
    }

    this->motor_right->setSpeed((int)(linear_speed + angular_speed));
    this->motor_left->setSpeed((int)(linear_speed - angular_speed));
}

void Kr1bou::set_PID_L(float Kp, float Ki, float Kd){
    this->KP_L = Kp;
    this->KI_L = Ki;
    this->KD_L = Kd;
}

void Kr1bou::set_PID_R(float Kp, float Ki, float Kd){
    this->KP_R = Kp;
    this->KI_R = Ki;
    this->KD_R = Kd;
}

