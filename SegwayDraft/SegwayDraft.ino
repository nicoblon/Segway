#include "config.h"

//Encoders
ESP32Encoder encoder1;
ESP32Encoder encoder2;

//**************************
#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

void setup() {

  // GENERATE THE LIST OF COMMANDS OF EACH PREDEFINED PATH
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

  encoder1.attachFullQuad(A1, A2);  //full quadrature so we divide by 4 in the angle
  encoder1.setCount(0);             //set the count of the 1st encoder to 0

  encoder2.attachFullQuad(A3, A4);
  encoder2.setCount(0);  //set the count of the 2nd encoder to 0

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  serverStuff();

  K_P = K_P_stable;
  K_D = K_D_stable;

  yaw_offset = -ypr.yaw;

  
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
  // put your main code here, to run repeatedly:
  //K_P = K_P_stable;
  //K_D = K_D_stable;

  // counting time
  t_start = micros();

  // check accelerometer
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  yaw = ypr.yaw - yaw_offset;

  // read encoders and convert to radians
  rad1 = -(encoder1.getCount() / 4) * 2 * pi / (32 * Rapport);
  rad2 = (encoder2.getCount() / 4) * 2 * pi / (32 * Rapport);

  //Position of the segway in [cm]
  pos_1 = rad1 * R;
  pos_2 = rad2 * R;
  pos = (pos_1 + pos_2) / 2;  //Average of the position of the 2 wheels
  yaw_wheels = (pos_1 - pos_2) / L * 180 / pi;

  // Calculating the current speed
  speed = (pos - prev_pos) / DeltaTime;
  // Updating buffer and calculating average speed
  updateSpeedBuffer(speed);                                  //adds value to speedBuffer list
  average_speed = averageNonZero(speedBuffer, BUFFER_SIZE);  // calculating average speed over the last 10 values

  prev_pos = pos;  // updating previous position variable

  // Read accelerometer and transforming into an angle
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
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

  /*
  if(settingsPath1){
    RightMotorAdjustment = 0.91;
    pitch_bias = 2.5;
  }else if(settingsPath2){
    RightMotorAdjustment = 0.88;
    pitch_bias = 2.8;
  }else if(settingsPath3){
    RightMotorAdjustment = 0.96;
    pitch_bias = 1.85;
    MaxSpeed = 0.025;
  }*/

  if(circuit){
    MaxSpeed = 0.058;
    pitch_bias = 3;
    RightMotorAdjustment = 0.91;
  }else{
    if(!settingsPath3){
      MaxSpeed = 0.015;
    }
    //pitch_bias = 2.2;
  }

  if (reset) {
    //encoder1.setCount(0);
    //encoder2.setCount(0);
    x_ref = pos;
    turn_cmmd = yaw_wheels;
    //rad1 = 0;
    //rad2 = 0;
    //pos_1 = 0;
    //pos_2 = 0;
    //pos = 0;

    //pitch_err=0;
    //pre_error=0;
    //sum_error=0;
    //sum_p_error=0;
    //sum_y_error=0;
    //sum_error_speed = 0;
    //previous_error_speed = 0;
    //previous_error = 0;
    numCommandsChosen = 2;
    cntr_yaw = 2;
    cntr_pos = 3;
    K_P = K_P_stable;
    K_D = K_D_stable;
    start = false;
    circuit = false;
    reset = false;
  } else {

    if (start) {
      K_P = K_P_move;
      K_D = K_D_move;
    } else {
      K_P = K_P_stable;
      K_D = K_D_stable;
    }

    /*if (x_ref != x_ref_prev) {
        hasReset = false;
        K_P = K_P_move;
        K_D = K_D_move;
        //MaxSpeed = 0.03;
        //encoder1.setCount(0);
        //encoder2.setCount(0);
        //rad1 = -(encoder1.getCount()/4)*2*pi / (32*Rapport); 
        //rad2 = (encoder2.getCount()/4)*2*pi / (32*Rapport); 

        //Position of the segway in [cm]
        //pos_1 = rad1*R;
        //pos_2 = rad2*R;
        //pos = (pos_1 + pos_2)/2;
        //yaw_wheels = (pos_1-pos_2)/L * 180/pi;
        //prev_pos=0;
      }
      /*if(abs(t) <= position_error){
        K_P = K_P_stable;
        K_D = K_D_stable;
      }*/



    prev_power = power;
    t = x_ref - pos;
    power = t - sgn(t) * position_error;

    // Check if need to reset variables
    /*if (abs(t) <= position_error && !hasReset) {
      resetVariables(t);
      hasReset = true;
    }else{
      if (x_ref != x_ref_prev) {
        hasReset = false;
        K_P = K_P_move;
        K_D = K_D_move;
        MaxSpeed = 0.03;
        encoder1.setCount(0);
        encoder2.setCount(0);
        rad1 = -(encoder1.getCount()/4)*2*pi / (32*Rapport); 
        rad2 = (encoder2.getCount()/4)*2*pi / (32*Rapport); 

        //Position of the segway in [cm]
        pos_1 = rad1*R;
        pos_2 = rad2*R;
        pos = (pos_1 + pos_2)/2;
        yaw_wheels = (pos_1-pos_2)/L * 180/pi;
        prev_pos=0;
      }
      if(average_speed < speed_err && abs(t) < position_error){
        K_P = K_P_stable;
        K_D = K_D_stable;
      }*/


    x_ref_prev = x_ref;
    float K_P_Speed = Kp_speed;
    float K_I_Speed = Ki_speed;
    float K_D_Speed = D_stop;

    // Calculate reference speed with respect to a reference position
    refSpeed = P_decreasing_speed(pos, x_ref, average_speed, K_P_Speed, K_I_Speed, K_D_Speed);

    // Setting PID constants (stability vs movement)
    /*if(abs(refSpeed) < speed_err){     // if the speed is less than x% of the maximum speed -> stabilize
      K_P = K_P_stable; 
      K_D = K_D_stable;
    }else{
      K_P= K_P_move; 
      K_D=K_D_move;
    }
    */
    pitch = ypr.pitch - pitch_bias;  // adjusting pitch with bias

    // Create an input spike when starting motion
    if (x_ref != x_ref_prev && prevSpeed != refSpeed) {
      x = D_Start(refSpeed, prevSpeed);
    } /*
    if(abs(t) <= position_error && !hasReset){
      x = Stop(D_stop, average_speed, prevSpeed);
      hasReset = true;
    }*/
    /*else if(abs(pos-x_ref)<position_error && average_speed>MinSpeed){
      x=D_stop(average_speed, prevSpeed);
    }*/
    else {
      x = PI_p_feedback(Kp_P, Kp_I, average_speed, refSpeed);  // calculating reference angle based on reference speed
    }                                                          //Add desired speed

    prevSpeed = average_speed;  // updating previous speed variable

    pitch_err = pitch + x;  // adding reference angle to current angle with bias

    x_cmmd = PID_feedback(pitch_err, K_P, K_I, K_D);  // Computes command to give to the motors (forwards/backwards motion only)
    yaw_cmmd = PI_y_feedback(Ky_P, Ky_I, turn_cmmd, yaw_wheels);

    avail_turn = (255 - abs(x_cmmd));

    if (abs(yaw_cmmd) > avail_turn) yaw_cmmd = sgn(yaw_cmmd) * avail_turn;

    Travel(x_cmmd, yaw_cmmd);  // Function that instructs motors what to do

    // time management, making every loop iteration exactly 10ms
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
