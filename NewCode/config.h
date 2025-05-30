// config.h
#ifndef CONFIG_H
#define CONFIG_H

#include <Adafruit_BNO08x.h> 
#include <Arduino.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define BNO08X_RESET -1 

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h> //Try also <ESPAsyncWebSrv.h> if it doesn't work
#include <ESP32Encoder.h>

extern ESP32Encoder encoder1;
extern ESP32Encoder encoder2;

extern AsyncWebServer server;
extern const char* ssid;
extern const char* password;

// Counters for path planning
int cntr_yaw = 2;
int cntr_pos = 3;

// PID values (text) - Shared variables
extern String P_val_S;
extern String P_val_M;
extern String I_val;
extern String D_val_S;
extern String D_val_M;
extern String PitchBias_val;
extern String Kp_P_val;
extern String Kp_I_val;
extern String Ky_P_val;
extern String Ky_I_val;
extern String LeftMotorAdjustment_val;
extern String RightMotorAdjustment_val;
extern String D_start_val;
extern String x_ref_val;
extern String Kp_speed_val;
extern String speed_err_val;
extern String position_error_val;
extern String Ki_speed_val;
extern String D_stop_val;
extern String turn_cmmd_val;
extern String MaxSpeed_val;

//---CONSTANTS---
  //Rayon roues
    const int R = 3.2; //[cm]
    const float L = 20.5;

    const float pi = 3.14159265;

  //Rapport de réduction du gearhead
    const int Rapport = 29; 
    
  //Pin du contrôle moteur
    const int M1A = 33; 
    const int M1B = 27; 
    const int M2A = 15; 
    const int M2B = 32;

  // Constantes de vitesse
    #define BUFFER_SIZE 10  // Store last 10 speed values
    int DeltaTime=10;
    float speedBuffer[BUFFER_SIZE] = {0};  // Initialize with zeros
    int speedIndex = 0;  // Track the position to overwrite
    float average_speed=0;
    float speed=0;

  // Controllers parameters
    float K_P = 0; //value that switches between K_P_stable and K_P_move, not web modifiable
    float K_P_stable = 7; //11
    float K_P_move = 7;
    float K_I = 0.6;
    float K_D = 0; //value that switches between K_D_stable and K_D_move, not web modifiable
    float K_D_stable = 21;
    float K_D_move = 21;
    float pitch_bias = 3.8; //change depending on center of gravity
    float Kp_P = 100;//0.5;
    float Kp_I = 25;//0.1;
    float Ky_P = 5;//0.5;
    float Ky_I = 0;//1;
    float MaxSpeed=0.035;
    float MinSpeed=0.02;
    float prevSpeed=0;
    float x=0;
    float t = 0;
    float Kp_speed=0.2;
    float Ki_speed=0.01;
    float x_ref = 0;
    float x_ref_prev = 0;
    float speed_err = 0.035;
    float refSpeed = 0;
    float position_error = 5;

    float timeOverflow=10000;

    float LeftMotorAdjustment = 0.89;
    float RightMotorAdjustment = 0.91;

    float D_start = 1000;
    float D_stop = 0;

    float sum_power = 0;
    float power = 0;
    float prev_power = 0;

    bool resetALL = false;
    bool hasReset = false;
    uint8_t resetCount = 0;


  // Tracking of error 
    float pitch_err=2.25;
    float pre_error=0; 
    float sum_error=0; 
    float sum_p_error=0;
    float sum_y_error=0;
    float sum_error_speed = 0;
    float previous_error = 0;
    float previous_error_speed = 0;

    float pos_ref = 0;
    float pitch = 0.0; 
    float pitch_ref = 0.0;
    float yaw_ref = 0.0;
    float yaw_offset = 0;
    
  // Declaring variables that used to be declared in PID/PD to make faster
    float K_prop;
    float K_int;
    float K_diff;
    float Kp_prop;
    float Kp_int;
    float Ky_prop;
    float Ky_int;
    
  // Declaring variables that used to be declared in void loop to make faster

    // To get the postion of each whell
    float rad1;
    float rad2;   
    float pos_1;
    float pos_2;
    float prev_pos=0;
    const int max_v = 255; 
    const int max_angle = 30;

    int x_cmmd;
    int x_cmmd_prev = 0;
    int avail_turn; //command left available to turn

    int turn_cmmd;
    int yaw_cmmd;

    int command;
    int l;
    int r;

    float pos;
    float yaw;
    float yaw_wheels; // yaw calculé avec la position des roues

    // Boolean used to reset all the quantities giving informations 
    //on the segway relative position
    bool reset = false;

    // Counters 
    int counterugh = 0;
    int tempCount = 0;

    // Boolean used to check if the robot started its trajectory after the initial balancing phase
    bool start = false;

    bool circuit = false;

    // Path points
    const int numPoints1 = 13;
    const int numPoints2 = 13;
    const int numPoints3 = 17;
    const int numPoints4 = 9;
    const int numPoints5 = 25;

    int numCommands1 = 0;
    int numCommands2 = 0;
    int numCommands3 = 0;
    int numCommands4 = 0;
    int numCommands5 = 0;

    int numCommandsChosen = 2;

    int *numCmd[5];

    // PID field
    const char* P_input_S = "KPS";
    const char* P_input_M = "KPM";
    const char* I_input = "KI";
    const char* D_input_S = "KDS";
    const char* D_input_M = "KDM";
    const char* PB_input = "PB";
    const char* KPpos_input = "KPp";
    const char* KIpos_input = "KIp";
    const char* KPyaw_input = "KPY";
    const char* KIyaw_input = "KIY";
    const char* LeftMotorAdjustment_input = "LMot";
    const char* RightMotorAdjustment_input = "RMot";
    const char* D_start_input = "Ds";
    const char* x_ref_input = "Xref";
    const char* Kp_speed_input = "KP_sp";
    const char* speed_err_input = "Sp_err";
    const char* position_error_input = "pos_err";
    const char* Ki_speed_input = "KI_sp";
    const char* D_stop_input = "Dsp";
    const char* turn_cmmd_input = "t_c";
    const char* MaxSpeed_input = "MaxS";

    //Timing terms
    unsigned long t_start;
    unsigned long t_end;
    unsigned long t_loop;

// Accelerometer variables
  Adafruit_BNO08x  bno08x(BNO08X_RESET); 
  sh2_SensorValue_t sensorValue; 

struct euler_t { 
  float yaw; 
  float pitch; 
  float roll; 
} ypr; 

struct Coordinates{
  int x;
  int y;
};

struct Output{
  float angle;
  int distance;
};

struct Coordinates path1[numPoints1] = {
  {0,0}, {50,0}, {16,12}, {15,48}, {-6,19}, {-40,29}, {-20, 0}, {-40,-29}, {-6,-19}, {15,-48}, {16,-12}, {50,0}, {16,12} 
};
struct Coordinates path2[numPoints2] = {
  {0,0}, {50,0}, {0,0}, {15,48}, {0,0}, {-40,29}, {0,0}, {-40,-29}, {0,0}, {15,-48}, {0,0}, {50,0}, {0,0},
};
struct Coordinates path3[numPoints3] = {
  {0,0}, {50,0}, {34,16}, {31,39}, {8,37}, {-11,49}, {-23,29}, {-45,22}, {-38,0}, {-45,-22}, {-23,-29}, {-11,-49}, {8,-37}, {31,-39}, {34,-16}, {50,0}, {34,16}
};
struct Coordinates path4[numPoints4] = {
  {0,0}, {50,0}, {5,9}, {-25,43}, {-10,0}, {-25,-43}, {5,-9}, {50,0}, {5,9}
};
struct Coordinates path5[numPoints5] = {
  {0,0}, {225,0}, {225,100}, {25,100}, {25,250}, {-125,250}, {-125, 125}, {-125,0},
  {0,0}, {225,0}, {225,100}, {25,100}, {25,250}, {-125,250}, {-125, 125}, {-125,0},
  {0,0}, {225,0}, {225,100}, {25,100}, {25,250}, {-125,250}, {-125, 125}, {-125,0}, {0,0}
};

struct Output commands1[2*numPoints1];
struct Output commands2[2*numPoints2];
struct Output commands3[2*numPoints3];
struct Output commands4[2*numPoints4];
struct Output commands5[2*numPoints5];

struct Output* chosenPathCommands;
struct Output* chosenCommands[5];

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = true);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false);
void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false); 
void setReports(sh2_SensorId_t reportType, long report_interval);
int sgn(int x);
float PI_p_feedback(float Kp_P, float Kp_I, float pos, float pos_ref);
int PID_feedback(float pitch_err, float K_P, float K_I, float K_D);
float D_Start(float v_ref, float v_prev);
int Stop(float D_stop, float current_speed, float previous_speed);
float P_decreasing_speed(float x, float x_ref, float speed, float K_P_Speed, float K_I_Speed, float K_D_Speed);
void resetVariables();
void Travel(int x_command, int turn_command);
void updateSpeedBuffer(float newSpeed);
float averageNonZero(float arr[], int size);
float PI_y_feedback(float Ky_P,float Ky_I, float yaw_ref, float yawIn);
void serverStuff();
float calculateAngle(int dx, int dy);
int calculateDistance(int dx, int dy);
void generateCommands(struct Coordinates points[], int numPoints, struct Output commands[], int *numCommands);
void funct_yaw_ref(struct Output command);
void funct_pos_ref(struct Output command);

#endif