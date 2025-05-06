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
  // put your setup code here, to run once:
  Serial.begin(115200); 
  delay(1000);
  Serial.println("Adafruit BNO08x test!");

  encoder1.attachFullQuad (A1, A2); //full quadrature so we divide by 4 in the angle
  encoder1.setCount (0); //set the count of the 1st encoder to 0

  encoder2.attachFullQuad (A3, A4);
  encoder2.setCount (0); //set the count of the 2nd encoder to 0
  
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
}

void loop() {
  // put your main code here, to run repeatedly:

  // counting time
  t_start=micros();

  // check accelerometer 
  if (bno08x.wasReset()) { 
    Serial.print("sensor was reset "); 
    setReports(reportType, reportIntervalUs); 
  } 

  // read encoders and convert to radians
  rad1 = -(encoder1.getCount()/4)*2*pi / (32*Rapport); 
  rad2 = (encoder2.getCount()/4)*2*pi / (32*Rapport); 

  //Position of the segway in [cm]
  pos_1 = rad1 * R;
  pos_2 = rad2 * R;
  pos = (pos_1 + pos_2)/2; //Average of the position of the 2 wheels

  // Calculating the current speed 
  speed= (pos-prev_pos)/DeltaTime;
  // Updating buffer and calculating average speed
  updateSpeedBuffer(speed); //adds value to speedBuffer list
  average_speed=averageNonZero(speedBuffer, BUFFER_SIZE); // calculating average speed over the last 10 values
  
  prev_pos=pos; // updating previous position variable

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
  
  prev_power = power;                   // updating previous power variable
  t = x_ref - pos;                      //position error
  power = t - sgn(t) * position_error;  // calculating power

  // Check if need to reset variables
  if (abs(t) <= position_error && !hasReset) {
    resetVariables();
    hasReset = true;
  }else{
    if (x_ref != x_ref_prev) {
      hasReset = false;
      K_P = K_P_move;
      K_D = K_D_move;
    }
    if (abs(pos)> abs(0.5 * x_ref) && !hasReset){
      MaxSpeed=0.02;
    }
    if (abs(t) <= position_error) {
      hasReset = true;
      MaxSpeed=0.035;

    }

    x_ref_prev = x_ref;

    // Calculate reference speed with respect to a reference position
    refSpeed = P_decreasing_speed(pos, x_ref);

    // Setting PID constants (stability vs movement)
    /*if(abs(refSpeed) < speed_err){     // if the speed is less than x% of the maximum speed -> stabilize
      K_P = K_P_stable; 
      K_D = K_D_stable;
    }else{
      K_P= K_P_move; 
      K_D=K_D_move;
    }
    */
    pitch = ypr.pitch - pitch_bias; // adjusting pitch with bias

    // Create an input spike when starting motion
    if(prevSpeed==0 && prevSpeed != refSpeed){
      x=D_Start(refSpeed, prevSpeed);
    }
    /*else if(abs(pos-x_ref)<position_error && average_speed>MinSpeed){
      x=D_stop(average_speed, prevSpeed);
    }*/
    else{
      x = PI_p_feedback(Kp_P, Kp_I, average_speed, refSpeed); // calculating reference angle based on reference speed
    }//Add desired speed 

    prevSpeed=average_speed; // updating previous speed variable

    pitch_err = pitch +x ; // adding reference angle to current angle with bias

    x_cmmd = PID_feedback(pitch_err, K_P, K_I, K_D); // Computes command to give to the motors (forwards/backwards motion only)

    yaw_wheels = asin((pos_1-pos_2)/L)* 180/pi;

    yaw_cmmd = PI_y_feedback(Ky_P, Ky_I, turn_cmmd, yaw_wheels);

    Travel(x_cmmd, yaw_cmmd); // Function that instructs motors what to do

    Serial.print("Kp_yaw: ");
    Serial.print(Ky_P);
    Serial.print(", Ki_yaw: ");
    Serial.println(Ky_I);

    // time management, making every loop iteration exactly 10ms
    t_end=micros();
    t_loop=t_end-t_start;
    while(t_loop > timeOverflow){
      timeOverflow+=100;
    }
    DeltaTime = timeOverflow/1000;
    delayMicroseconds(timeOverflow - t_loop);
  }  
  t_end=micros();
}
