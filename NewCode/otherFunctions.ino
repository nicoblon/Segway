#include "config.h"

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) { 

  float sqr = sq(qr); 
  float sqi = sq(qi); 
  float sqj = sq(qj); 
  float sqk = sq(qk); 

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)); 
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)); 
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)); 

  if (degrees){ 
    ypr->yaw *= RAD_TO_DEG; 
    ypr->pitch *= RAD_TO_DEG; 
    ypr->roll *= RAD_TO_DEG; 
  } 
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees) { 
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees); 
} 

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees) { 
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees); 
} 

void setReports(sh2_SensorId_t reportType, long report_interval) { 
  Serial.println("Setting desired reports"); 
  if (! bno08x.enableReport(reportType, report_interval)) { 
    Serial.println("Could not enable stabilized remote vector"); 
  } 
} 

//basic sign function, it returns 1 if x is positive, and -1 if not. 
int sgn(int x){ 
  if (x >= 0){ 
    return 1; 
  }else{ 
    return -1; 
  } 
} 

//Function which compute the error on position
//and return the new pitch reference
float PI_p_feedback(float Kp_P, float Kp_I, float pos, float pos_ref) {
  float error_pos = pos - pos_ref;
  if (abs(error_pos) > 1) sum_p_error += error_pos;  //add the current error to the previous one
  sum_p_error *= 0.98;                               //leaky integrator
  if (abs(sum_p_error) > 10) {
    sum_p_error = constrain(sum_p_error, -10, 10);
  }
  Kp_prop = Kp_P * error_pos;   //Proportional part
  Kp_int = Kp_I * sum_p_error;  //Integral part

  int pitch_cmmd = round(Kp_prop + Kp_int);
  if (abs(pitch_cmmd) > max_angle) pitch_cmmd = sgn(pitch_cmmd) * max_angle;  //limit the feedback from min -255 to the max 255

  return (pitch_cmmd);
}

int PID_feedback(float pitch_err, float K_P, float K_I, float K_D) {
  float error = pitch_err;
  if (abs(error) > 0.1) sum_error += error;  //add the current error to the previous one
  sum_error *= 0.98;                         //leaky integrator

  K_prop = K_P * error;                //proportional part of the controller
  K_int = K_I * sum_error;             //integral part of the controller
  K_diff = K_D * (error - pre_error);  //differential part of the controller

  pre_error = error;  //previous error update

  int feedback = round(K_prop + K_int + K_diff);  //round up the feedback to an integer

  if (abs(feedback) > max_v) feedback = sgn(feedback) * max_v;  //limit the feedback from min -255 to the max 255

  return (feedback);
}

float D_Start(float v_ref, float v_prev) {
  float error_speed = v_ref - v_prev;
  int feedback = round(D_start * (error_speed / DeltaTime));
  return (feedback);
}

int Stop(float D_stop, float current_speed, float previous_speed) {
  return  -sgn(current_speed) * round(D_stop * abs(current_speed));
}

float P_decreasing_speed(float x, float x_ref, float speed, float K_P_Speed, float K_I_Speed, float K_D_Speed) {
  float error = x_ref - x;

  // Stop if close enough to target
  if (abs(error) <= position_error){
    sum_error_speed = 0;
    return 0;
  }

  // Only accumulate if not saturated (anti-windup)
  float proportional = Kp_speed * error;
  float integral = Ki_speed * sum_error_speed;
  float derivative = - D_stop * speed;
  float feedback = proportional + integral + derivative;

  if(abs(feedback) < MaxSpeed){
    sum_error_speed += error;
    sum_error_speed *= 0.95;  // decay after update
  }

  // Saturate output
  if (abs(feedback) > MaxSpeed) {
    return sgn(feedback) * MaxSpeed;
  }

  return feedback;
}

void resetVariables() {
  if (hasReset) {
    return;
  }
  x_ref = position_error;
  encoder1.setCount(0);
  encoder2.setCount(0);
  /*for(int i = 0; i <= 9; i++){
    updateSpeedBuffer(0);
  }*/
  pos_1 = 0;
  pos_2 = 0;
  pos = 0;
  x_ref_prev = x_ref;
  prev_pos = 0;
  K_P = K_P_stable;
  K_D = K_D_stable;
  //power = 0;
  //sum_power = 0;
  //sum_error = 0;
  //sum_p_error = 0;
  refSpeed = 0;
  resetCount++;
  MaxSpeed = 0.02;

  return;
}

// Function used to turn
void Travel(int x_command, int turn_command) {  // instruction to the motors with the command
  int l = x_command - turn_command;             //left wheel
  int r = x_command + turn_command;             //right wheel
  
  if (l >= 0 && r >= 0) {
    analogWrite(M1A, 0);
    analogWrite(M1B, r);
    analogWrite(M2A, l);
    analogWrite(M2B, 0);
  } else if (l < 0 && r >= 0) {
    analogWrite(M1A, 0);
    analogWrite(M1B, r);
    analogWrite(M2A, 0);
    analogWrite(M2B, -l);
  } else if (l >= 0 && r < 0) {
    analogWrite(M1A, -r);
    analogWrite(M1B, 0);
    analogWrite(M2A, l);
    analogWrite(M2B, 0);
  } else if (l < 0 && r < 0) {
    analogWrite(M1A, -r);
    analogWrite(M1B, 0);
    analogWrite(M2A, 0);
    analogWrite(M2B, -l);
  }
}

//function that updates the oldest term in the speed array
void updateSpeedBuffer(float newSpeed) {
  speedBuffer[speedIndex] = newSpeed;           // Replace oldest value
  speedIndex = (speedIndex + 1) % BUFFER_SIZE;  // Move to next index (circular behavior)
}

//Function that gets the average of an array
float averageNonZero(float arr[], int size) {
  float sum = 0, count = 0;
  for (int i = 0; i < size; i++) (arr[i] != 0) ? (sum += arr[i], count++) : 0;
  return count ? sum / count : 0.0;
}

// Function PI_y_feedback that compute and define the turn command
float PI_y_feedback(float Ky_P, float Ky_I, float yaw_ref, float yawIn) {
  float error_yaw = yaw_ref - yawIn;
  if (abs(error_yaw) > 0.1) sum_y_error += error_yaw;  //add the current error to the previous one
  sum_y_error *= 0.9;     

  /*if (abs(sum_y_error)>10){
    sum_y_error=constrain(sum_y_error,-10,10);
  }*/

  Ky_prop = Ky_P * error_yaw;
  Ky_int = Ky_I * sum_y_error;
  int feedback = round(Ky_prop + Ky_int);
  if (abs(feedback) > max_v) feedback = sgn(feedback) * max_v;  //limit the feedback from min -255 to the max 255
  return feedback;
  //if (abs(yaw_cmmd)>180) yaw_cmmd=180
}

float calculateAngle(int dx, int dy){
  //float angleAbs;
  float angle = 0;

  if(dy != 0){
    angle = atan2(dy, dx) * 180 / pi;
  }else{
    if(dx < 0){
      angle = 180;
    }else{
      angle = 0;
    }
  }

  /*if(!circuit){
    if(angle < -180){
      angle += 360;
    }else if(angle > 180){
      angle -= 360;
    }
  }*/

  return angle;
}

int calculateDistance(int dx, int dy){
  int distance = sqrt(pow(dx,2) + pow(dy,2));
  return distance;
}

void generateCommands(struct Coordinates points[], int numPoints, struct Output commands[], int *numCommands){

  //initialization
  commands[0].angle = 0; // Segway aligned with the reference axis (x)
	commands[1].distance = 0; // not relevant

  *numCommands = 2;
  struct Coordinates currentPosition = points[0];
  float previousAngle = 0;

  for(int i = 1; i < numPoints; i++){
    int dx = points[i].x - points[i-1].x;
    int dy = points[i].y - points[i-1].y;

    int distance = calculateDistance(dx, dy);
    float angle = calculateAngle(dx, dy);

    if(!circuit){
      if((angle - previousAngle) < -90){
        distance *= -1;
        angle += 180;
      }
      if((angle - previousAngle) > 90){
        distance *= -1;
        angle -= 180;
      }
    }
    

    // Storing the commands
    if(circuit){
      commands[*numCommands].angle = angle;
    }else{
      commands[*numCommands].angle = angle - previousAngle;
    }
    
    (*numCommands)++;
    previousAngle = angle;

    commands[*numCommands].distance = distance;
    (*numCommands)++;
  }
}

void funct_yaw_ref(struct Output command){      // doesn't really need to be a function honestly
  turn_cmmd += command.angle * LeftMotorAdjustment;
}

void funct_pos_ref(struct Output command){     // doesn't really need to be a function honestly
  x_ref += command.distance * RightMotorAdjustment;
}