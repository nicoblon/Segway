#include "config.h"

//Encoders
ESP32Encoder encoder1;
ESP32Encoder encoder2;

// UNTESTED CODE
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;

  // USE THIS INSTEAD IF NOT WORKING!
  /*
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable) 
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV; 
  long reportIntervalUs = 2000; 
  */

void setup() {

  // GENERATE THE LIST OF COMMANDS FOR EACH PREDEFINED PATH
  generateCommands(path1, numPoints1, commands1, &numCommands1);
  generateCommands(path2, numPoints2, commands2, &numCommands2);
  generateCommands(path3, numPoints3, commands3, &numCommands3);
  generateCommands(path4, numPoints4, commands4, &numCommands4);
  generateCommandsCircuit(path5, numPoints5, commands5, &numCommands5);
  generateCommandsCircuit(path6, numPoints6, commands6, &numCommands6);

  numCmd[0] = &numCommands1;
  numCmd[1] = &numCommands2;
  numCmd[2] = &numCommands3;
  numCmd[3] = &numCommands4;
  numCmd[4] = &numCommands5;
  numCmd[5] = &numCommands6;

  chosenCommands[0] = &commands1[0];
  chosenCommands[1] = &commands2[0];
  chosenCommands[2] = &commands3[0];
  chosenCommands[3] = &commands4[0];
  chosenCommands[4] = &commands5[0];
  chosenCommands[5] = &commands6[0];

  Serial.begin(115200);
  delay(1000);

  Serial.println("Adafruit BNO08x test!");

  // Encoder set-up
  encoder1.attachFullQuad(A1, A2);  //full quadrature so we divide by 4 in the angle
  encoder1.setCount(0);             //set the count of the 1st encoder to 0

  encoder2.attachFullQuad(A3, A4);
  encoder2.setCount(0);  //set the count of the 2nd encoder to 0

  // Try to initialize accelerometer!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  // Setting up webserver functions
  serverStuff();

  // Setting Kp and Kd to stable
  K_P = K_P_stable;
  K_D = K_D_stable;
  
  // Print the commands of a specific path -> for verification of the commands
  for (int i = 0; i < 2 * numPoints5; i++) {
    Serial.print("Command ");
    Serial.print(i);
    Serial.print(": Angle = ");
    Serial.print(commands5[i].angle);
    Serial.print(", Distance = ");
    Serial.println(commands5[i].distance);
  }
}

void loop() {
  // counting time to ensure every loop iteration is identical in duration
  t_start = micros();

  // check accelerometer
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  // read encoders and convert to radians
  rad1 = -(encoder1.getCount() / 4) * 2 * pi / (32 * Rapport);
  rad2 = (encoder2.getCount() / 4) * 2 * pi / (32 * Rapport);

  // Position of the segway in [cm]
  pos_1 = rad1 * R;
  pos_2 = rad2 * R;
  pos = (pos_1 + pos_2) / 2;  //Average of the position of the 2 wheels

  // Yaw calculated from encoder positions -> used in PI controller for turning instead of the accelerometer yaw 
  yaw_wheels = (pos_1 - pos_2) / L * 180 / pi;

  // Calculating the current speed
  speed = (pos - prev_pos) / DeltaTime;
  // Updating buffer and calculating average speed
  updateSpeedBuffer(speed);                                  //adds value to speedBuffer list
  average_speed = averageNonZero(speedBuffer, BUFFER_SIZE);  // calculating average speed over the last 10 values

  prev_pos = pos;  // updating previous position variable

  // Read accelerometer and transforming into an angle in degrees
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        break;
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    static long last = 0;
    long now;
  }

  // Updating the reference yaw and position with the commands for the paths
  if (numCommandsChosen > 2) {
    start = true;
    if (cntr_yaw == 2) {
      funct_yaw_ref(*(chosenPathCommands + cntr_yaw));  //[0] and [1] are ignored as they are initializations (cntr_yaw_pos starts at 2)
      cntr_yaw += 2;
    }
    if ((abs(turn_cmmd - yaw_wheels) < 2) && cntr_pos == 3) {
      funct_pos_ref(*(chosenPathCommands + cntr_pos));
      cntr_pos += 2;
    }
    if ((abs(x_ref - pos) < position_error) && (cntr_yaw < cntr_pos) && (cntr_yaw < numCommandsChosen)) {  //When position error is small, the segway is at the correct position so it starts to turn in the direction of the next point
      funct_yaw_ref(*(chosenPathCommands + cntr_yaw));
      cntr_yaw += 2;
    }
    if ((abs(turn_cmmd - yaw_wheels) < 2) && (cntr_pos < cntr_yaw) && (cntr_pos < numCommandsChosen)) {  //When yaw error is small, the segway is at the correct angle so it starts to move in the direction of the next point
      funct_pos_ref(*(chosenPathCommands + cntr_pos));
      cntr_pos += 2;
    }
    if ((cntr_yaw == numCommandsChosen) && (cntr_pos == numCommandsChosen + 1) && (abs(x_ref - pos) < 5)) {
      reset = true;
    }
  }

  // Adjusting some parameters depending on which path was chosen (may need to be adjusted further)
  if(settingsPath1){
    distanceAdjustmentFactor = 0.91;
    pitch_bias = 2.5;
  }else if(settingsPath2){
    distanceAdjustmentFactor = 0.88;
    pitch_bias = 2.8;
  }else if(settingsPath3){
    distanceAdjustmentFactor = 0.96;
    pitch_bias = 1.85;
    MaxSpeed = 0.025;
  }

  if(circuit){
    MaxSpeed = 0.07;
    pitch_bias = 3;
    distanceAdjustmentFactor = 0.91;
  }else{
    if(!settingsPath3){
      MaxSpeed = 0.015;
    }
  }

  // Checking reset condition
  if (reset) {
    x_ref = pos;
    turn_cmmd = yaw_wheels;
    numCommandsChosen = 2;
    cntr_yaw = 2;
    cntr_pos = 3;
    K_P = K_P_stable;
    K_D = K_D_stable;
    start = false;
    circuit = false;
    reset = false;
  } else {

    // if motion -> Kp_move, Kd_move, otherwise if stationary -> Kp_stable, Kd_stable
    if (start) {
      K_P = K_P_move;
      K_D = K_D_move;
    } else {
      K_P = K_P_stable;
      K_D = K_D_stable;
    }

    // Updating previous reference position
    x_ref_prev = x_ref;

    // Calculate reference speed with respect to a reference position
    refSpeed = P_decreasing_speed(pos, x_ref, average_speed, Kp_speed, Ki_speed);

    pitch = ypr.pitch - pitch_bias;  // adjusting pitch with bias

    // Create an input spike when starting motion
    if (x_ref != x_ref_prev && prevSpeed != refSpeed) {
      pitch_ref = D_Start(refSpeed, prevSpeed);
    }else{
      pitch_ref = PI_p_feedback(Kp_P, Kp_I, average_speed, refSpeed);  // calculating reference angle based on reference speed
    }

    prevSpeed = average_speed;  // updating previous speed variable

    pitch_err = pitch + pitch_ref;  // adding reference angle to current angle with bias

    x_cmmd = PID_feedback(pitch_err, K_P, K_I, K_D);  // Computes command to give to the motors (forwards/backwards motion only)
    yaw_cmmd = PI_y_feedback(Ky_P, Ky_I, turn_cmmd, yaw_wheels);  // Computes command to give to the motors to induce rotation

    // Making sure that the rotational command never overtakes the stability commands to ensure the Segway is always able to stay upright
    avail_turn = (255 - abs(x_cmmd));
    if (abs(yaw_cmmd) > avail_turn) yaw_cmmd = sgn(yaw_cmmd) * avail_turn;

    Travel(x_cmmd, yaw_cmmd);  // Function that instructs motors what to do

    // time management, making every loop iteration exactly the same duration (> 10 ms)
    t_end = micros();
    t_loop = t_end - t_start;
    while (t_loop > timeOverflow) {
      timeOverflow += 100;
    }
    DeltaTime = timeOverflow / 1000;
    delayMicroseconds(timeOverflow - t_loop);
  }
  t_end = micros();
}
