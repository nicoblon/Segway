#include "config.h"

//basic sign function, it returns 1 if x is positive, and -1 if not. 
int sgn(int x){ 
  if (x >= 0){ 
    return 1; 
  }else{ 
    return -1; 
  } 
} 

// Functions that transform the accelerometer output into degrees
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

// Function for accelerometer -> from previous Segway code
void setReports(sh2_SensorId_t reportType, long report_interval) { 
  Serial.println("Setting desired reports"); 
  if (! bno08x.enableReport(reportType, report_interval)) { 
    Serial.println("Could not enable stabilized remote vector"); 
  } 
} 

// PI controller of the position w.r.t. a reference position
float PI_p_feedback(float Kp_P, float Kp_I, float pos, float pos_ref) {
  float error_pos = pos - pos_ref;
  if (abs(error_pos) > 1) sum_p_error += error_pos;  //add the current error to the previous one if the error is greater than 1
  sum_p_error *= 0.98;                               //leaky integrator -> works kind of like an anti-windup
  if (abs(sum_p_error) > 10) {                       //constrain the error to not overload the PI control
    sum_p_error = constrain(sum_p_error, -10, 10);
  }
  Kp_prop = Kp_P * error_pos;   //Proportional part
  Kp_int = Kp_I * sum_p_error;  //Integral part

  int pitch_cmmd = round(Kp_prop + Kp_int);
  if (abs(pitch_cmmd) > max_angle) pitch_cmmd = sgn(pitch_cmmd) * max_angle;  //limit the feedback from min -255 to the max 255

  return (pitch_cmmd);
}

// PID controller of the pitch w.r.t. a reference angle
int PID_feedback(float pitch_err, float K_P, float K_I, float K_D) {
  float error = pitch_err;
  if (abs(error) > 0.1) sum_error += error;  //add the current error to the previous one if the error is greater than 0.1
  sum_error *= 0.98;                         //leaky integrator -> works kind of like an anti-windup

  K_prop = K_P * error;                //proportional part of the controller
  K_int = K_I * sum_error;             //integral part of the controller
  K_diff = K_D * (error - pre_error);  //differential part of the controller

  pre_error = error;  //previous error update

  int feedback = round(K_prop + K_int + K_diff);  //round up the feedback to an integer

  if (abs(feedback) > max_v) feedback = sgn(feedback) * max_v;  //limit the feedback from min -255 to the max 255

  return (feedback);
}

// Function that creates an impulse when starting motion
float D_Start(float v_ref, float v_prev) {
  float error_speed = v_ref - v_prev;
  int feedback = round(D_start * (error_speed / DeltaTime));
  return (feedback);
}

// PI controller of the speed w.r.t. a reference position -> calculates the necessary speed to reach the wanted position
float PI_decreasing_speed(float x, float x_ref, float K_P_Speed, float K_I_Speed) {
  float error = x_ref - x;

  // Stop if close enough to target
  if (abs(error) <= position_error){
    sum_error_speed = 0;
    return 0;
  }

  // Calculating control parameters
  float proportional = Kp_speed * error;
  float integral = Ki_speed * sum_error_speed;
  float feedback = proportional + integral;

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

// Function for movement, linear and rotational
void Travel(int x_command, int turn_command) {
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

// Updating average_speed array
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

// PI controller of the yaw w.r.t a reference angle
float PI_y_feedback(float Ky_P, float Ky_I, float yaw_ref, float yawIn) {
  float error_yaw = yaw_ref - yawIn;
  if (abs(error_yaw) > 0.1) sum_y_error += error_yaw;  //add the current error to the previous one
  sum_y_error *= 0.9;     

  Ky_prop = Ky_P * error_yaw;
  Ky_int = Ky_I * sum_y_error;
  int feedback = round(Ky_prop + Ky_int);
  if (abs(feedback) > max_v) feedback = sgn(feedback) * max_v;  //limit the feedback from min -255 to the max 255
  return feedback;
}

// Function that calculates the angle between 2 points for the star-shaped paths
float calculateAngle(int dx, int dy){
  float angle = 0;

  if(dy != 0){
    angle = atan2(dy, dx) * 180 / pi;
  }else{
    if(dx < 0){
      angle = 180;
    }
    else{
      angle = 0;
    }
  }

  return angle;
}

// Function that calculates the angle between 2 points for the circuit.
  // this is a different function than the paths because we wanted to avoid changing the movement direction from forwards to backwards to avoid losing time
float calculateAngleCircuit(int dx, int dy){
  float angle;
  angle = atan2(dy, dx)*180/pi;
  return angle;
}

// Function to calculate the distance between 2 points 
int calculateDistance(int dx, int dy){
  int distance = sqrt(pow(dx,2) + pow(dy,2));
  return distance;
}

// Normalization of an angle between -180 and 180 degrees
float normalizeAngle(float angle) {
    while (angle <= -180) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
}

// Creating the list of commands to follow the star-shaped paths, angle and distance
void generateCommands(struct Coordinates points[], int numPoints, struct Output commands[], int *numCommands){
  // the x-axis is facing in the direction of the Segway (forward motion)
  // the y-axis is +90° w.r.t. the x-axis (counter-clockwise)

  //initialization
  commands[0].angle = 0; // Segway aligned with the reference axis (x)
	commands[1].distance = 0; // not relevant
  
  *numCommands = 2; // position in the commands array -> starts at 2 because the initial angle and distance values are set to zero

  struct Coordinates currentPosition = points[0]; // setting the current position to the start of the path
  float previousAngle = 0;

  for(int i = 1; i < numPoints; i++){
    // Calculating the distances over the x and y axes between 2 points
    int dx = points[i].x - points[i-1].x;
    int dy = points[i].y - points[i-1].y;

    // Finding the distance and the angle between the points
    int distance = calculateDistance(dx, dy);
    float angle = calculateAngle(dx, dy);

    // if the angle is not contained in [-90, 90] then make the Segway go backwards to avoid making large turns
    if((angle - previousAngle) < -90){
      distance *= -1;
      angle += 180;
    }
    if((angle - previousAngle) > 90){
      distance *= -1;
      angle -= 180;
    }

    // Storing the commands
    commands[*numCommands].angle = angle - previousAngle;
    
    (*numCommands)++;
    previousAngle = angle;

    commands[*numCommands].distance = distance;
    (*numCommands)++;
  }
}
// Creating the list of commands to follow the circuit, angle and distance
  // Used another function to make sure the Segway is always facing forwards, to avoid losing time turning around and changing the direction of motion
void generateCommandsCircuit(struct Coordinates points[], int numPoints, struct Output commands[], int *numCommands){
  // the x-axis is facing in the direction of the Segway (forward motion)
  // the y-axis is +90° w.r.t. the x-axis (counter-clockwise)

  //initialization
  commands[0].angle = 0; // Segway aligned with the reference axis (x)
	commands[1].distance = 0; // not relevant

  *numCommands = 2; // position in the commands array -> starts at 2 because the initial angle and distance values are set to zero

  struct Coordinates currentPosition = points[0]; // setting the current position to the starting point of the circuit
  float previousAngle = 0;

  for(int i = 1; i < numPoints; i++){
    // Calculating the distances over the x and y axes between 2 points
    int dx = points[i].x - points[i-1].x;
    int dy = points[i].y - points[i-1].y;

    // Finding the distance and the angle between the points, making sure that the Segway will always move forward
    int distance = calculateDistance(dx, dy);
    float angle = calculateAngleCircuit(dx, dy);
    float deltaAngle = normalizeAngle(angle - previousAngle);

    // Storing the commands
    commands[*numCommands].angle = deltaAngle;
    previousAngle = angle;
    
    (*numCommands)++;

    commands[*numCommands].distance = distance;
    (*numCommands)++;
  }
}

// Updating the reference yaw angle with the commands generated
void funct_yaw_ref(struct Output command){      
  turn_cmmd += command.angle * angleAdjustmentFactor;
}

// Updating the reference position with the commands generated
void funct_pos_ref(struct Output command){
  x_ref += command.distance * distanceAdjustmentFactor;
}