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

    this->initial_x = 0;
    this->initial_y = 0;
    this->x = 0;
    this->y = 0;
    this->a = 0;
}

void Kr1bou::updateOdometry() {
    double r1 = this->moteur1->SpeedCurrent;
    double r2 = this->moteur2->SpeedCurrent;
    dt1 = (this->moteur1->dt+this->moteur2->dt)/2;
    if (dt1 > 1000000) return;
    double v = (r1+r2)/2.0;
    double w = (r2-r1)/weeldistance;
    da = w*dt1/1000000.0;
    x += v*cos(a+da/2.)*dt1/1000000.0;
    y += v*sin(a+da/2.)*dt1/1000000.0;
    a += da;
    a = mod2pi(a);
    /*Serial.print(a);
    Serial.print(",");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(this->objectif_x);
    Serial.print(",");
    Serial.println(this->objectif_y);*/
}

void Kr1bou::updateMotors(bool allow_backward) {
    //objectif à atteindre
    this->etat = IN_PROGESS;
    float x2 = this->objectif_x;
    float y2 = this->objectif_y;

    float destination_angle = atan2f(y2 - this->y, x2 - this->x);

    //float destination_angle = this->objectif_theta;
    
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
    float target_angle = destination_angle + atan(dist_to_line * 3.5f) * (is_on_right_side ? -1.0f : 1.0f); //7.0f

    float dx_base = x2 - pos.x;
    float dy_base = y2 - pos.y;
    //float dx_target = x2 + GOTO_DELTA * cos(destination_angle) - pos.x;
    //float dy_target = y2 + GOTO_DELTA * sin(destination_angle) - pos.y;

    float dist_to_base = sqrt(dx_base * dx_base + dy_base * dy_base);

    //float angle_to_target = atan2f(dy_target, dx_target);
   // float diff_angle_direction = AngleDiffRad(angle_to_target,destination_angle);


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
    bool backward = abs(angle_diff) > M_PI / 2;
    //bool backward = false;


    backward = backward && allow_backward;
    if (backward)
    {
        /* If going backward, we target the opposite angle */
        angle_diff = AngleDiffRad(target_angle + M_PI, this->a);
    }

    /* Computing the two speed components */
    float forward_speed = 0.0f;
    float turn_speed = 0.0f;
    float speed_limit = (backward ? WHEEL_BACKWARD_SPEED : WHEEL_FORWARD_SPEED);

    ////////////////////////////////// SI High Speed multiplié ici la speed limit par WHEEL_HIGHSPEED_FACTOR 

    forward_speed = speed_limit * (1.0f - abs(angle_diff) / (M_PI / 2));

    if (m_goto_base_reached)
    {
        forward_speed *= 0.1f;
        //Serial.print("m_goto_base_reached");
    }
    if (backward)
    {
        forward_speed = -forward_speed;
    }

    /* Turn speed is proportional to the angle difference (just the opposite of the forward speed!) */
    turn_speed = abs(angle_diff) / (M_PI / 2);
    turn_speed *= (backward ? WHEEL_TURN_SPEED_BACKWARD : WHEEL_TURN_SPEED_FORWARD);
    turn_speed *= angle_diff > 0 ? 1.0f : -1.0f; // Turn left or right

    /* If the robot speed limit is the limiting factor instead of our parameters (because of a slow
    * down), we reduce everything proportionally so that we keep the same turn/forward ratio. */
    float reduction_factor = 1.0f;
    float left_speed = reduction_factor * (forward_speed + turn_speed);
    float right_speed = reduction_factor * (forward_speed - turn_speed);

    /////////////////////
    //goto_over = true;
    /////////////////////
    
    /* Note: we reset the PIDs if the GOTO is over, but we don't know if that's really needed */
    if (this->etat == READY)
    {
        left_speed = 0.0f;
        right_speed = 0.0f;
    }
    this->moteur1->setSpeedConsign(left_speed);
    this->moteur2->setSpeedConsign(right_speed);
    this->moteur1->updateSpeedPID();
    this->moteur2->updateSpeedPID();
    this->last_left_speed = left_speed;
    this->last_right_speed = right_speed;

}


void Kr1bou::updateMotors_2(bool allow_backward){

    this->etat = IN_PROGESS;

    float x1 = this->objectif_x;
    float y1 = this->objectif_y;
    float x0 = this->initial_x;
    float y0 = this->initial_y;
    
    //Serial.println("updateMotors_2 :: 1");
    float alpha_D = atan2f(y1 - y0, x1 - x0);

    if (need_angle_correction){
        need_angle_correction = !angleCorection(alpha_D);
        return;
    }

    Vec2f u = Vec2f{cosf(alpha_D), sinf(alpha_D)};

    //Serial.println("updateMotors_2 :: 2");

    float x = this->x;
    float y = this->y;
    float alpha = this->a;
    
    float xH = 0;
    float yH = 0;

    if ((x0*y1-x1*y0) != 0){

        float a = (y1-y0)/(x0*y1-x1*y0);
        float b = (x0-x1)/(x0*y1-x1*y0);
        
        if(b >= 0.1){
            xH = (u.x*x + u.y*y - u.y/b)/(u.x - u.y*a/b);
            yH = (1-a*xH)/b;
        }
        else{
            yH = (u.x*x + u.y*y - u.x/a)/(u.y - u.x*b/a);
            xH = (1-b*yH)/a;
        }
    }
    else{
        if (x0 < x1+0.1 && x0 > x1-0.1){
            xH = x1;
            yH = y;
        }
        else{
            xH = x;
            yH = y0;
        }
    }
    //Serial.println("updateMotors_2 :: 3");

    //definition de la position du robot par rapport à la droite (D) (utilisation du signe du produit vectoriel)
    bool gauche = (u.x * (y-yH) - u.y * (x-xH)) > 0;

    //distance entre le robot et le point H
    float distToLigne = sqrtf((x - xH) * (x - xH) + (y - yH) * (y - yH));
    distToLigne *= (gauche ? -1 : 1); //on prend en compte le signe de la distance

    //Serial.println("updateMotors_2 :: 4");

    //distance entre le robot et le point d'arrivé
    float distToArrive = sqrtf((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y));
    if (distToArrive < DISTANCE_PRECISION)
    {
        this->etat = READY;
        return;
    }

    //Serial.println("updateMotors_2 :: 5");

    //angle entre la droite (D) et la droite (R) (robot)
    float angle = AngleDiffRad(alpha_D, alpha);

    //Serial.println("updateMotors_2 :: 6");

    //calcule de la vitesse de rotation et de translation
    float w = KLigne*distToLigne + KAngle*(angle+distToLigne/10);
    float v = WHEEL_FORWARD_SPEED - abs(w * weeldistance /2);
    if (v < 0)v = 0;

    //calcule de la vitesse des roues
    float v1 = v + weeldistance * w / 2;
    float v2 = v - weeldistance * w / 2;

    //Serial.println("updateMotors_2 :: 7");

    //envoie des consignes de vitesse
    this->moteur1->setSpeedConsign(v1);
    this->moteur2->setSpeedConsign(v2);

    //Serial.println("updateMotors_2 :: 8");
    this->resetMotorIntegrator();
    this->moteur1->updateSpeedPID();
    this->moteur2->updateSpeedPID();

    /*Serial.print("v1 : ");
    Serial.print(v1);
    Serial.print(" | v2 : ");
    Serial.println(v2);*/
}

void Kr1bou::setObjective(float x, float y, float theta) {
    this->objectif_x = x;
    this->objectif_y = y;
    this->objectif_theta = theta;
    this->initial_x = this->x;
    this->initial_y = this->y;
    need_angle_correction = true;
    //Serial.print("nouvelle objectif : ");
    //Serial.print(x);
    //Serial.print(",");
    //Serial.println(y);
}

void Kr1bou::resetMotorIntegrator() {
    this->moteur1->errorSum = 0;
    this->moteur2->previousSpeed = 0;
    this->moteur2->errorSum = 0;
    this->moteur2->previousSpeed = 0;
}

bool Kr1bou::angleCorection(float angle_obj){
    float angle = AngleDiffRad(this->a, angle_obj);
    if (abs(angle) > ANGLE_PRECISION){
        float w = angle * KP_R;
        float v = 0;
        if (abs(w) > 0.5){
            v = 0;
        }
        this->moteur1->setSpeedConsign(v + weeldistance * w / 2);
        this->moteur2->setSpeedConsign(v - weeldistance * w / 2);
        this->moteur1->updateSpeedPID();
        this->moteur2->updateSpeedPID();   
        return false;
    }
    this->resetMotorIntegrator();
    return true;
}