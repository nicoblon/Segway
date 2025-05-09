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
  if (abs(error_pos)>1) sum_p_error+=error_pos; //add the current error to the previous one
  sum_p_error *= 0.98; //leaky integrator
  if (abs(sum_p_error)>10){
    sum_p_error=constrain(sum_p_error,-10,10);
  }
  Kp_prop = Kp_P * error_pos; //Proportional part
  Kp_int=Kp_I * sum_p_error; //Integral part 

  int pitch_cmmd = round(Kp_prop + Kp_int);
  if (abs(pitch_cmmd)>max_v) pitch_cmmd=sgn(pitch_cmmd)*max_v; //limit the feedback from min -255 to the max 255 

  return (pitch_cmmd);
}

int PID_feedback(float pitch_err, float K_P, float K_I, float K_D){ 
  float error = pitch_err;
  if (abs(error)>0.1) sum_error+=error; //add the current error to the previous one
  sum_error *= 0.98; //leaky integrator
  
  K_prop=K_P*error; //proportional part of the controller
  K_int=K_I*sum_error; //integral part of the controller 
  K_diff=K_D*(error-pre_error); //differential part of the controller  
  
  pre_error=error; //previous error update  
  
  int feedback=round(K_prop+K_int+K_diff); //round up the feedback to an integer 
  
  if (abs(feedback)>max_v) feedback=sgn(feedback)*max_v; //limit the feedback from min -255 to the max 255 
 
  return (feedback);  
}

float D_Start(float v_ref, float v_prev){
  float error_speed= v_ref-v_prev;
  int feedback= round(D_start*(error_speed/DeltaTime));
  return(feedback);
}

/*float D_stop(float , float){
  //ajouter que en qd power change de signe on met un spike qui est en fonction de la vitesse (refSpeed?)
  int x = 1;
  if(sgn(power)>0){
    x = -1
  }
  int feedback = sgn(x)*round(D_stop*()); // on doit faire un pic en fonction de la vitesse, mais on prend speed ou refSpeed? Aussi, pour le derivative term, on prend quoi comme erreur?
  return feedback;
}*/

// Function that makes the speed decrease as we approach the wanted position
float P_decreasing_speed(float x, float x_ref){
  if(abs(t) <= position_error) power = 0;
  sum_power*=0.95;
  sum_power+=power;
  float feedback = power*Kp_speed+Ki_speed*sum_power;
  if (abs(feedback)>MaxSpeed) return (sgn(feedback)*MaxSpeed);
  if ((abs(feedback) < MinSpeed) && power!=0) return (sgn(feedback)*MinSpeed);
  else return (feedback);
}

void resetVariables() {
  if (hasReset) {
    return;
  }
  x_ref = position_error;
  x_ref_prev = x_ref;
  encoder1.setCount(0);
  encoder2.setCount(0);
  /*for(int i = 0; i <= 9; i++){
    updateSpeedBuffer(0);
  }*/
  pos_1 = 0;
  pos_2 = 0;
  pos = 0;
  prev_pos = 0;
  K_P = K_P_stable;
  K_D = K_D_stable;
  //power = 0;
  //sum_power = 0;
  //sum_error = 0;
  //sum_p_error = 0;
  refSpeed=0;
  resetCount++;
  MaxSpeed = 0.02;
  return;
}

// Function used to turn
void Travel(int x_command, int turn_command) {  // instruction to the motors with the command
  l = x_command - turn_command / 2;             //left wheel
  r = x_command + turn_command / 2;             //right wheel
  if (l >= 0 && r >= 0) {
    analogWrite(M1A, 0);
    analogWrite(M2A, l * LeftMotorAdjustment);
    analogWrite(M1B, r * RightMotorAdjustment);
    analogWrite(M2B, 0);
  } else if (l < 0 && r >= 0) {
    analogWrite(M1A, 0);
    analogWrite(M2A, 0);
    analogWrite(M1B, r * RightMotorAdjustment);
    analogWrite(M2B, -l * LeftMotorAdjustment);
  } else if (l >= 0 && r < 0) {
    analogWrite(M1A, l * LeftMotorAdjustment);
    analogWrite(M2A, -r * RightMotorAdjustment);
    analogWrite(M1B, 0);
    analogWrite(M2B, 0);
  } else if (l < 0 && r < 0) {
    analogWrite(M1A, -r * RightMotorAdjustment);
    analogWrite(M2A, 0);
    analogWrite(M1B, 0);
    analogWrite(M2B, -l * LeftMotorAdjustment);
  }
}

//function that updates the oldest term in the speed array
void updateSpeedBuffer(float newSpeed) {
    speedBuffer[speedIndex] = newSpeed;  // Replace oldest value
    speedIndex = (speedIndex + 1) % BUFFER_SIZE;  // Move to next index (circular behavior)
}

//Function that gets the average of an array
float averageNonZero(float arr[], int size) {
    float sum = 0, count = 0;
    for (int i = 0; i < size; i++) (arr[i] != 0) ? (sum += arr[i], count++) : 0;
    return count ? sum / count : 0.0;
}

// Function PI_y_feedback that compute and define the turn command
float PI_y_feedback(float Ky_P,float Ky_I, float yaw_ref, float yaw_cmmd){
  float error_yaw=yaw_cmmd-yaw_ref; //yaw : turn command, angle in degres [°]
  if (abs(error_yaw)>0.1) sum_y_error+=error_yaw; //add the current error to the previous one
  sum_y_error *= 0.9 ;//leaky integrator
  if (abs(sum_y_error)>10){
    sum_y_error=constrain(sum_y_error,-10,10);
  }
  Ky_prop=Ky_P*error_yaw;
  Ky_int=Ky_I*sum_y_error;
  int feedback=round(Ky_prop+K_int);
  if(abs(feedback)>MaxSpeed) return (sgn(feedback)*MaxSpeed);
  if(abs(feedback)<MinSpeed) return (sgn(feedback)*MinSpeed);
  else return feedback;
  //if (abs(yaw_cmmd)>180) yaw_cmmd=180
}
