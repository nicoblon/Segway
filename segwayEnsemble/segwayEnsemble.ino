// config.h
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

AsyncWebServer server(80);
const char* ssid = "IPhone de Nicolas";
const char* password = "nicolebg";

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

// Counters for path planning
int cntr_yaw = 2;
int cntr_pos = 3;

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
    float K_P_stable = 14;
    float K_P_move = 7;
    float K_I = 0.6;
    float K_D = 0; //value that switches between K_D_stable and K_D_move, not web modifiable
    float K_D_stable = 21;
    float K_D_move = 21;
    float pitch_bias = 4; //change depending on center of gravity
    float Kp_P = 100;//0.5;
    float Kp_I = 25;//0.1;
    float Ky_P = 5;//0.5;
    float Ky_I = 0;//1;
    float MaxSpeed=0.03;
    float MinSpeed=0.02;
    float prevSpeed=0;
    float x=0;
    float t = 0;
    float Kp_speed=0.2;
    float Ki_speed=0.01;
    float x_ref = 0;
    float x_ref_prev = 0;
    float speed_err = 0.025;
    float refSpeed = 0;
    float position_error = 5;

    float timeOverflow=10000;

    float LeftMotorAdjustment = 1;  //0.975
    float RightMotorAdjustment = 1;

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

    // Path points
    const int numPoints1 = 13;
    const int numPoints2 = 13;
    const int numPoints3 = 17;
    const int numPoints4 = 9;
    const int numPoints5 = 8;

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

    //***************************
  // PID values (text)
  String P_val_S = String(K_P_stable);
  String P_val_M = String(K_P_move);
  String I_val = String(K_I);
  String D_val_S = String(K_D_stable);
  String D_val_M = String(K_D_move);
  String PitchBias_val = String(pitch_bias);
  String Kp_P_val = String(Kp_P);
  String Kp_I_val = String(Kp_I);
  String Ky_P_val = String(Ky_P);
  String Ky_I_val = String(Ky_I);
  String LeftMotorAdjustment_val = String(LeftMotorAdjustment);
  String RightMotorAdjustment_val = String(RightMotorAdjustment);
  String D_start_val = String(D_start);
  String x_ref_val = String(x_ref);
  String Kp_speed_val = String(Kp_speed);
  String speed_err_val = String(speed_err);
  String position_error_val = String(position_error);
  String Ki_speed_val = String(Ki_speed);
  String D_stop_val = String(D_stop);
  String turn_cmmd_val = String(turn_cmmd);
  String MaxSpeed_val = String(MaxSpeed);

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
  {0,0}, {50,0}, {0,0}, {15,48}, {0,0}, {-40,29}, {0,0}, {-40,29}, {0,0}, {15,-48}, {0,0}, {50,0}, {0,0},
};
struct Coordinates path3[numPoints3] = {
  {0,0}, {50,0}, {34,16}, {31,39}, {8,37}, {-11,49}, {-23,29}, {-45,22}, {-38,0}, {-45,-22}, {-23,-29}, {-11,-49}, {8,-37}, {31,-39}, {34,-16}, {50,0}, {34,16}
};
struct Coordinates path4[numPoints4] = {
  {0,0}, {50,0}, {5,9}, {-25,43}, {-10,0}, {-25,-43}, {5,-9}, {50,0}, {5,9}
};
struct Coordinates path5[numPoints5] = {
  {0,0}, {225,0}, {225,100}, {25,100}, {25,250}, {-125,250}, {-125,0}, {0,0}
};

struct Output commands1[2*numPoints1];
struct Output commands2[2*numPoints2];
struct Output commands3[2*numPoints3];
struct Output commands4[2*numPoints4];
struct Output commands5[2*numPoints5];

struct Output* chosenPathCommands;
struct Output* chosenCommands[5];

// Replaces placeholder with button section in your web page
String processor(const String& var){
  if (var == "PPS"){
    return P_val_S;
  }else if (var == "PPM"){
    return P_val_M;
  }else if (var == "II"){
    return I_val;
  }else if (var == "DDS"){
    return D_val_S;
  }else if (var == "DDM"){
    return D_val_M;
  }else if (var == "BB"){
    return PitchBias_val;
  }else if (var == "KPp"){
    return Kp_P_val;
  }else if (var == "KIp"){
    return Kp_I_val;
  }else if (var == "Ky_P"){
    return Ky_P_val;
  }else if (var == "Ky_I"){
    return Ky_I_val;
  }else if (var == "x_ref"){
    return x_ref_val;
  }else if (var == "LeftMotorAdjustment"){
    return LeftMotorAdjustment_val;
  }else if (var == "RightMotorAdjustment"){
    return RightMotorAdjustment_val;
  }else if (var == "D_start"){
    return D_start_val;
  }else if (var == "Kp_speed"){
    return Kp_speed_val;
  }else if(var == "speed_err"){
    return speed_err_val;
  }else if(var == "position_error"){
    return position_error_val;
  }else if (var == "Ki_speed"){
    return Ki_speed_val;
  }else if(var == "D_stop"){
    return D_stop_val;
  }else if(var == "turn_cmmd"){
     return turn_cmmd_val;
  }else if(var == "MaxSpeed"){
    return MaxSpeed_val;
  }return String();
}


// HTML root page
const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>PID web tuning</title>
      <style>
        body {
          font-family: Arial, sans-serif;
          background-color: #ffe6f0; /* Light pink background */
          margin: 20px;
        }

        h2 {
          text-align: center;
          color: #d63384; /* Deep pink */
        }

        .param {
          display: flex;
          align-items: center;
          justify-content: space-between;
          background: #fff0f5; /* Very light pink */
          padding: 10px;
          margin: 10px 0;
          border-radius: 8px;
          box-shadow: 0 2px 5px rgba(214, 51, 132, 0.2); /* Pink shadow */
        }

        .param span {
          flex: 2;
          font-size: 14px;
          padding-right: 10px;
          color: #880e4f;
        }

        .param input {
          flex: 1;
          padding: 5px;
          font-size: 14px;
          border: 1px solid #f48fb1;
          border-radius: 4px;
          margin-right: 10px;
        }

        .param button {
          background-color: #e91e63; /* Pink button */
          border: none;
          padding: 8px 16px;
          font-size: 14px;
          border-radius: 4px;
          cursor: pointer;
          color: white;
        }

        .container {
          display: flex;
          justify-content: center;
          align-items: center;
          flex-wrap: wrap;
          margin-top: 20px;
        }

        .button {
          font-size: 18px;
          padding: 12px 20px;
          margin: 6px;
          border-radius: 8px;
          background-color: #ec407a; /* Lighter pink */
          color: white;
          border: none;
          cursor: pointer;
        }

        .container-flex {
          display: flex;
          justify-content: space-between;
          align-items: flex-start;
          gap: 20px;
        }

        .column {
          flex: 1;
          padding: 10px;
          box-sizing: border-box;
        }

        .column h3 {
          margin-top: 0;
          color: #ad1457;
        }

        .section {
          margin-bottom: 20px;
        }

        /* Path buttons aligned horizontally */
        .path-buttons {
          display: flex;
          justify-content: center;
          gap: 10px;
          margin: 20px 0;
        }

        /* Directional control in cross layout */
        .control-cross {
          display: grid;
          grid-template-areas:
            ". forward ."
            "left stop right"
            ". backward .";
          gap: 10px;
          justify-content: center;
          align-items: center;
          margin-top: 20px;
        }

        .control-cross .forward   { grid-area: forward; }
        .control-cross .backward  { grid-area: backward; }
        .control-cross .left      { grid-area: left; }
        .control-cross .right     { grid-area: right; }
        .control-cross .stop      { grid-area: stop; }

        .control-cross button {
          width: 100px;
          height: 40px;
        }
      </style>
    </head>
    <body>
    
      <h2>PID Web Tuning</h2>
      <div class="container-flex">

        <!-- LEFT COLUMN: PID PARAMETERS -->
        <div class="column">
          <h3>PID Parameters</h3>

          <div class="section">
            <strong>Stabilization PID</strong>

            <p><span id="textPropValS">Prop. Gain (current: %PPS%) </span>
            <input type="number" id="KPS" value="%PPS%" min="-1000" max="1000" step="1">
            <button onclick="implement_P_S()">Submit</button></p>

            <p><span id="textIntVal">Integral Gain (current: %II%) </span>
            <input type="number" id="KI" value="%II%" min="0" max="10" step="0.05"> 
            <button onclick="implement_I()">Submit</button></p>

            <p><span id="textDerValS">Der Gain (current: %DDS%) </span>
            <input type="number" id="KDS" value="%DDS%" min="0" max="100" step="1">
            <button onclick="implement_D_S()">Submit</button></p>
          </div>

          <div class="section">
            <strong>Motion PID</strong>

            <p><span id="textPropValM">Prop. Gain (current: %PPM%) </span>
            <input type="number" id="KPM" value="%PPM%" min="0" max="100" step="1">
            <button onclick="implement_P_M()">Submit</button></p>

            <p><span id="textDerValM">Der Gain (current: %DDM%) </span>
            <input type="number" id="KDM" value="%DDM%" min="0" max="100" step="1"> 
            <button onclick="implement_D_M()">Submit</button></p>
          </div>

          <div class="section">
            <strong>Speed PID</strong>

            <p><span id="textKp_speedVal">Prop. Gain (current: %Kp_speed%) </span>
            <input type="number" id="KP_sp" value="%Kp_speed%" min="0" max="10000" step="1"> 
            <button onclick="implement_Kp_speed()">Submit</button></p>

            <p><span id="textKi_speedVal">Integral Gain (current: %Ki_speed%) </span>
            <input type="number" id="KI_sp" value="%Ki_speed%" min="0" max="10000" step="1"> 
            <button onclick="implement_Ki_speed()">Submit</button></p>
          </div>

          <div class="section">
            <strong>Position PID</strong>

            <p><span id="textKPpVal">Prop. Gain (current: %KPp%) </span>
            <input type="number" id="KPp" value="%KPp%" min="0" max="100" step="1"> 
            <button onclick="implement_KpP()">Submit</button></p>
            
            <p><span id="textKIpVal">Integral Gain (current: %KIp%) </span>
            <input type="number" id="KIp" value="%KIp%" min="0" max="10" step="0.05"> 
            <button onclick="implement_KpI()">Submit</button></p>
          </div>

          <div class="section">
            <strong>Yaw PID</strong>

            <p><span id="textKPyVal">Prop. Gain (current: %Ky_P%) </span>
            <input type="number" id="KPY" value="%Ky_P%" min="0" max="100" step="1"> 
            <button onclick="implement_KyP()">Submit</button></p>

            <p><span id="textKIyVal">Integral Gain (current: %Ky_I%) </span>
            <input type="number" id="KIY" value="%Ky_I%" min="0" max="10" step="0.05">
             <button onclick="implement_KyI()">Submit</button></p>
          </div>

          <div class="section">
            <strong>Misc. Parameters</strong>

            <p><span id="textPitchBiasVal">Pitch Bias (current: %BB%) </span>
            <input type="number" id="PB" value="%BB%" min="-45" max="45" step="0.1"> 
            <button onclick="implement_B()">Submit</button></p>
            
            <p><span id="textLeftMotorAdjustmentVal">L Motor Adj. (current: %LeftMotorAdjustment%) </span>
            <input type="number" id="LMot" value="%LeftMotorAdjustment%" min="0" max="1" step="0.01"> 
            <button onclick="implement_LeftMotorAdjustment()">Submit</button></p>
            
            <p><span id="textRightMotorAdjustmentVal">R Motor Adj. (current: %RightMotorAdjustment%) </span>
            <input type="number" id="RMot" value="%RightMotorAdjustment%" min="0" max="1" step="0.01"> 
            <button onclick="implement_RightMotorAdjustment()">Submit</button></p>
          </div>
        </div>

        <!-- MIDDLE COLUMN: LIVE DATA -->
        <div class="column">
          <h3>Live Data</h3>
          <p>Yaw (from wheels): <span id="yawWheelsVal">Loading...</span> deg</p>
          <p>Yaw (from accel.): <span id="yawVal">Loading...</span> deg </p>
          <p>Average speed: <span id="average_speedVal">Loading...</span></p>
          <p>Position error: <span id="position_errorVal">Loading...</span></p>
          <p>Pitch: <span id="pitchVal">Loading...</span> deg</p>
          <p>x_ref: <span id="x_refVal">Loading...</span></p>
          <p>turn_command: <span id="turn_cmmdVal">Loading...</span></p>
          <p>pos: <span id="posVal">Loading...</span> cm</p>

          <div class="section">
            <strong>Thresholds & Errors</strong>

            <p><span id="textD_startVal">D_start (current: %D_start%) </span>
            <input type="number" id="Ds" value="%D_start%" min="0" max="10000" step="10"> 
            <button onclick="implement_D_start()">Submit</button></p>
            
            <p><span id="textD_stopVal">D_stop (current: %D_stop%) </span>
            <input type="number" id="Dsp" value="%D_stop%" min="0" max="10000" step="10"> 
            <button onclick="implement_D_stop()">Submit</button></p>
            
            <p><span id="textspeed_errVal">Speed Error (current: %speed_err%) </span>
            <input type="number" id="Sp_err" value="%speed_err%" min="0" max="10" step="0.01"> 
            <button onclick="implement_speed_err()">Submit</button></p>
            
            <p><span id="textposition_errorVal">Position Error (current: %position_error%) </span>
            <input type="number" id="pos_err" value="%position_error%" min="0" max="100" step="1"> 
            <button onclick="implement_position_error()">Submit</button></p>

            <p><span id="textMaxSpeedVal">Max Speed (current: %MaxSpeed%) </span>
            <input type="number" id="MaxS" value="%MaxSpeed%" min="0" max="10" step="0.001"> 
            <button onclick="implement_MaxSpeed()">Submit</button></p>
          </div>
        </div>

        <!-- RIGHT COLUMN: CONTROLS -->
        <div class="column">
          <h3>Control Panel</h3>

          <div class="section">
            <p><span id="textx_refVal">Reference Pos. (current: %x_ref%) </span>
            <input type="number" id="Xref" value="%x_ref%" min="-1000" max="1000" step="50"> 
            <button onclick="implement_x_ref()">Submit</button></p>

            <p><span id="textturn_cmmdVal">Turn Command (current: %turn_cmmd%) </span>
            <input type="number" id="t_c" value="%turn_cmmd%" min="-180" max="180" step="1"> 
            <button onclick="implement_turn_cmmd()">Submit</button></p>
          </div>

          <div class="path-buttons">
            <strong>Path Commands</strong><br>
            <button class="button" onclick="sendcmmd('1')">Path 1</button>
            <button class="button" onclick="sendcmmd('2')">Path 2</button>
            <button class="button" onclick="sendcmmd('3')">Path 3</button>
            <button class="button" onclick="sendcmmd('4')">Path 4</button>
            <button class="button" onclick="sendcmmd('5')">Path 5</button>
          </div>

          <div class="control-cross">
            <strong>Manual Control</strong><br>
            <button class="button forward" onclick="sendcmmd('W')">Forward</button><br>
            <button class="button left" onclick="sendcmmd('A')">Left</button>
            <button class="button stop" onclick="sendcmmd('X')">Stop</button>
            <button class="button right" onclick="sendcmmd('D')">Right</button><br>
            <button class="button backward" onclick="sendcmmd('S')">Backward</button>
          </div>
        </div>
      </div>

    <script>

      function implement_turn_cmmd(){
         var turn_cmmd_val = document.getElementById("t_c").value;
         document.getElementById("textturn_cmmdVal").innerHTML = "Turn Command (current: " + turn_cmmd_val + ") ";
         console.log(turn_cmmd_val);
         var xhr = new XMLHttpRequest();
         xhr.open("GET", "/turn_cmmd?t_c="+turn_cmmd_val, true);
         xhr.send();
      }

      function implement_P_S(){
        var P_val_S = document.getElementById("KPS").value;
        document.getElementById("textPropValS").innerHTML = "Prop. Gain (current: " + P_val_S + ") ";
        console.log(P_val_S);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/proportionalS?KPS="+P_val_S, true);
        xhr.send();
      }

      function implement_P_M(){
        var P_val_M = document.getElementById("KPM").value;
        document.getElementById("textPropValM").innerHTML = "Prop. Gain (current: " + P_val_M + ") ";
        console.log(P_val_M);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/proportionalM?KPM="+P_val_M, true);
        xhr.send();
      }

      function implement_I(){
        var I_val = document.getElementById("KI").value;
        document.getElementById("textIntVal").innerHTML = "Integral gain (current: " + I_val + ") ";
        console.log(I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/integral?KI="+I_val, true);
        xhr.send();
      }


      function implement_D_S(){
        var D_val_S = document.getElementById("KDS").value;
        document.getElementById("textDerValS").innerHTML = "Der Gain (current: " + D_val_S + ") ";
        console.log(D_val_S);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/derivativeS?KDS="+D_val_S, true);
        xhr.send();
      }

      function implement_D_M(){
        var D_val_M = document.getElementById("KDM").value;
        document.getElementById("textDerValM").innerHTML = "Der Gain (current: " + D_val_M + ") ";
        console.log(D_val_M);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/derivativeM?KDM="+D_val_M, true);
        xhr.send();
      }

      function implement_B(){
        var PitchBias_val = document.getElementById("PB").value;
        document.getElementById("textPitchBiasVal").innerHTML = "Pitch Bias (current: " + PitchBias_val + ") ";
        console.log(PitchBias_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/pitchbias?PB="+PitchBias_val, true);
        xhr.send();
      }


      function implement_KpP(){
        var Kp_P_val = document.getElementById("KPp").value;
        document.getElementById("textKPpVal").innerHTML = "Prop. Gain (current: " + Kp_P_val + ") ";
        console.log(Kp_P_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kppos?ET="+Kp_P_val, true);
        xhr.send();
      }

      function implement_KpI(){
        var Kp_I_val = document.getElementById("KIp").value;
        document.getElementById("textKIpVal").innerHTML = "Integral Gain (current: " + Kp_I_val + ") ";
        console.log(Kp_I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kipos?ET="+Kp_I_val, true);
        xhr.send();
      }

      function implement_KyP(){
        var Ky_P_val = document.getElementById("KPY").value;
        document.getElementById("textKPyVal").innerHTML = "Prop. Gain (current: " + Ky_P_val + ") ";
        console.log(Ky_P_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kpyaw?KPY="+Ky_P_val, true);
        xhr.send();
      }

      function implement_KyI(){
        var Ky_I_val = document.getElementById("KIY").value;
        document.getElementById("textKIyVal").innerHTML = "Integral Gain (current: " + Ky_I_val + ") ";
        console.log(Ky_I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kiyaw?KIY="+Ky_I_val, true);
        xhr.send();
      }

      function implement_x_ref(){
          var x_ref_val = document.getElementById("Xref").value;
          document.getElementById("textx_refVal").innerHTML = "Reference Pos. (current: " + x_ref_val + ") ";
          console.log(x_ref_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/xref?Xref="+x_ref_val, true);
          xhr.send();
      }

      function implement_Kp_speed(){
          var Kp_speed_val = document.getElementById("KP_sp").value;
          document.getElementById("textKp_speedVal").innerHTML = "Prop. Gain (current: " + Kp_speed_val + ") ";
          console.log(Kp_speed_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/kpspeed?KP_sp="+Kp_speed_val, true);
          xhr.send();
      }

      function implement_Ki_speed(){
          var Ki_speed_val = document.getElementById("KI_sp").value;
          document.getElementById("textKi_speedVal").innerHTML = "Integral Gain (current: " + Ki_speed_val + ") ";
          console.log(Ki_speed_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/kispeed?KI_sp="+Ki_speed_val, true);
          xhr.send();
      }

      function implement_LeftMotorAdjustment(){
          var LeftMotorAdjustment_val = document.getElementById("LMot").value;
          document.getElementById("textLeftMotorAdjustmentVal").innerHTML = "L Motor Adj. (current: " + LeftMotorAdjustment_val + ") ";
          console.log(LeftMotorAdjustment_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/leftmotoradjustment?LMot="+LeftMotorAdjustment_val, true);
          xhr.send();
      }

      function implement_RightMotorAdjustment(){
          var RightMotorAdjustment_val = document.getElementById("RMot").value;
          document.getElementById("textRightMotorAdjustmentVal").innerHTML = "R Motor Adj. (current: " + RightMotorAdjustment_val + ") ";
          console.log(RightMotorAdjustment_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/rightmotoradjustment?RMot="+RightMotorAdjustment_val, true);
          xhr.send();
      }

      function implement_D_start(){
          var D_start_val = document.getElementById("Ds").value;
          document.getElementById("textD_startVal").innerHTML = "D_start (current: " + D_start_val + ") ";
          console.log(D_start_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/d_start?Ds="+D_start_val, true);
          xhr.send();
      }

      function implement_D_stop(){
          var D_stop_val = document.getElementById("Dsp").value;
          document.getElementById("textD_stopVal").innerHTML = "D_stop (current: " + D_stop_val + ") ";
          console.log(D_stop_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/d_stop?Dsp="+D_stop_val, true);
          xhr.send();
      }

      function implement_speed_err(){
          var speed_err_val = document.getElementById("Sp_err").value;
          document.getElementById("textspeed_errVal").innerHTML = "Speed Error (current: " + speed_err_val + ") ";
          console.log(speed_err_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/Speed_err?Sp_err="+speed_err_val, true);
          xhr.send();
      }

      function implement_position_error(){
          var position_error_val = document.getElementById("pos_err").value;
          document.getElementById("textposition_errorVal").innerHTML = "Position Error (current: " + position_error_val + ") ";
          console.log(position_error_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/Position_error?pos_err="+position_error_val, true);
          xhr.send();
      }

      function implement_MaxSpeed(){
          var MaxSpeed_val = document.getElementById("MaxS").value;
          document.getElementById("textMaxSpeedVal").innerHTML = "Max Speed (current: " + MaxSpeed_val + ") ";
          console.log(MaxSpeed_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/MaxSpeed?MaxS="+MaxSpeed_val, true);
          xhr.send();
      }

      function updateData(){
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/data", true);
        xhr.onreadystatechange = function(){
            if(xhr.readyState == 4 && xhr.status == 200){
                var obj = JSON.parse(xhr.responseText);
                document.getElementById("yawWheelsVal").innerHTML = obj.yaw_wheels.toFixed(2);
                document.getElementById("yawVal").innerHTML = obj.ypr_yaw.toFixed(2);
                document.getElementById("average_speedVal").innerHTML = obj.average_speed.toFixed(2);
                document.getElementById("position_errorVal").innerHTML = obj.position_error.toFixed(0);
                document.getElementById("pitchVal").innerHTML = obj.ypr_pitch.toFixed(2);
                document.getElementById("x_refVal").innerHTML = obj.x_ref.toFixed(0);
                document.getElementById("turn_cmmdVal").innerHTML = obj.turn_cmmd.toFixed(0);
                document.getElementById("posVal").innerHTMML = obj.position.toFixed(0);
            }
        };
        xhr.send();
      }

      // Refresh every 200 milliseconds (adjust if needed)
      setInterval(updateData, 100);


      function sendcmmd(cmmd){
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/cmmd?cmd=" + cmmd, true);
            xhr.send();
      }

    </script>
    </body>
  </html>
)rawliteral";

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
  if (abs(pitch_cmmd) > max_v) pitch_cmmd = sgn(pitch_cmmd) * max_v;  //limit the feedback from min -255 to the max 255

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

  if(angle < -180){
    angle += 360;
  }else if(angle > 180){
    angle -= 360;
  }

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

void funct_yaw_ref(struct Output command){      // doesn't really need to be a function honestly
  turn_cmmd += command.angle * 0.9;
}

void funct_pos_ref(struct Output command){     // doesn't really need to be a function honestly
  x_ref += command.distance * 0.92;
}


void setup(){

  // GENERATE THE LIST OF COMMANDS OF EACH PREDEFINED PATH
  generateCommands(path1, numPoints1, commands1, &numCommands1);
  generateCommands(path2, numPoints2, commands2, &numCommands2);
  generateCommands(path3, numPoints3, commands3, &numCommands3);
  generateCommands(path4, numPoints4, commands4, &numCommands4);
  generateCommands(path5, numPoints5, commands5, &numCommands5);

  numCmd[0] = &numCommands1;
	numCmd[1] = &numCommands2;
	numCmd[2] = &numCommands3;
	numCmd[3] = &numCommands4;
  numCmd[4] = &numCommands5;

  chosenCommands[0] = &commands1[0];
	chosenCommands[1] = &commands2[0];
  chosenCommands[2] = &commands3[0];
	chosenCommands[3] = &commands4[0];
	chosenCommands[4] = &commands5[0];

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


   // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Print ESP Local IP Address
  Serial.println(WiFi.localIP());
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/proportional?KP=<inputMessage>
  server.on("/proportionalS", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(P_input_S)) {
      inputMessage = request->getParam(P_input_S)->value();
      P_val_S = inputMessage;
      K_P_stable = P_val_S.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });//gets K_P_stable

  server.on("/proportionalM", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(P_input_M)) {
      inputMessage = request->getParam(P_input_M)->value();
      P_val_M = inputMessage;
      K_P_move = P_val_M.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });//Get K_P_move

  server.on("/integral", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(I_input)) {
      inputMessage = request->getParam(I_input)->value();
      I_val = inputMessage;
      K_I = I_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });//K_I

  server.on("/derivativeS", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(D_input_S)) {
      inputMessage = request->getParam(D_input_S)->value();
      D_val_S = inputMessage;
      K_D_stable = D_val_S.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });
  //K_D_stable

  server.on("/derivativeM", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(D_input_M)) {
      inputMessage = request->getParam(D_input_M)->value();
      D_val_M = inputMessage;
      K_D_move = D_val_M.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });//

  // Route for updating Kp_P
  server.on("/kppos", HTTP_GET, [](AsyncWebServerRequest* request){
    if (request->hasParam("ET")) {
      Kp_P = request->getParam("ET")->value().toFloat();
      Kp_P_val = String(Kp_P);
    }
    request->send(200);
  });

  // Route for updating Kp_I
  server.on("/kipos", HTTP_GET, [](AsyncWebServerRequest* request){
    if (request->hasParam("ET")) {
      Kp_I = request->getParam("ET")->value().toFloat();
      Kp_I_val = String(Kp_I);
    }
    request->send(200);
  });

  // Send a GET request to <ESP_IP>/pitchbias?PB=<inputMessage>
  server.on("/pitchbias", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/pitchbias?PB=<inputMessage>
    if (request->hasParam(PB_input)) {
      inputMessage = request->getParam(PB_input)->value();
      PitchBias_val = inputMessage;
      pitch_bias = PitchBias_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/kpyaw", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(KPyaw_input)) {
      inputMessage = request->getParam(KPyaw_input)->value();
      Ky_P_val = inputMessage;
      Ky_P = Ky_P_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/kiyaw", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(KIyaw_input)) {
      inputMessage = request->getParam(KIyaw_input)->value();
      Ky_I_val = inputMessage;
      Ky_I = Ky_I_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/xref", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(x_ref_input)) {
      inputMessage = request->getParam(x_ref_input)->value();
      x_ref_val = inputMessage;
      x_ref = x_ref_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/kpspeed", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(Kp_speed_input)) {
      inputMessage = request->getParam(Kp_speed_input)->value();
      Kp_speed_val = inputMessage;
      Kp_speed = Kp_speed_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/kispeed", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(Ki_speed_input)) {
      inputMessage = request->getParam(Ki_speed_input)->value();
      Ki_speed_val = inputMessage;
      Ki_speed = Ki_speed_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/leftmotoradjustment", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(LeftMotorAdjustment_input)) {
      inputMessage = request->getParam(LeftMotorAdjustment_input)->value();
      LeftMotorAdjustment_val = inputMessage;
      LeftMotorAdjustment = LeftMotorAdjustment_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/rightmotoradjustment", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(RightMotorAdjustment_input)) {
      inputMessage = request->getParam(RightMotorAdjustment_input)->value();
      RightMotorAdjustment_val = inputMessage;
      RightMotorAdjustment = RightMotorAdjustment_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/d_start", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(D_start_input)) {
      inputMessage = request->getParam(D_start_input)->value();
      D_start_val = inputMessage;
      D_start = D_start_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/d_stop", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(D_stop_input)) {
      inputMessage = request->getParam(D_stop_input)->value();
      D_stop_val = inputMessage;
      D_stop = D_stop_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/Speed_err", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(speed_err_input)) {
      inputMessage = request->getParam(speed_err_input)->value();
      speed_err_val = inputMessage;
      speed_err = speed_err_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/Position_error", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(position_error_input)) {
      inputMessage = request->getParam(position_error_input)->value();
      position_error_val = inputMessage;
      position_error = position_error_val.toFloat();
    }
    else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/plain", "OK");
  });

  server.on("/turn_cmmd", HTTP_GET, [] (AsyncWebServerRequest *request){
     String inputMessage;
     if(request->hasParam(turn_cmmd_input)) {    
       inputMessage = request->getParam(turn_cmmd_input)->value();
       turn_cmmd_val = inputMessage;
       turn_cmmd = turn_cmmd_val.toFloat();
     }
     else{
       inputMessage = "No message sent";
     }
     request->send(200, "text/plain", "OK");
  });

  server.on("/MaxSpeed", HTTP_GET, [] (AsyncWebServerRequest *request){
     String inputMessage;
     if(request->hasParam(MaxSpeed_input)) {    
       inputMessage = request->getParam(MaxSpeed_input)->value();
       MaxSpeed_val = inputMessage;
       MaxSpeed = MaxSpeed_val.toFloat();
     }
     else{
       inputMessage = "No message sent";
     }
     request->send(200, "text/plain", "OK");
  });

  /*server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"yaw_wheels\":" + String(yaw_wheels) + 
                  ",\"ypr_yaw\":" + String(yaw) +
                  ",\"average_speed\":" + String(average_speed) +
                  ",\"position_error\":" + String(position_error) +
                  ",\"ypr_pitch\":" + String(ypr.pitch) + 
                  ",\"x_ref\":" + String(x_ref) + 
                  ",\"turn_cmmd\":" + String(turn_cmmd) +
                  ",\"position\":" + String(pos) + "}";
    request->send(200, "application/json", json);
  });*/

  server.on("/cmmd", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("cmd")) {
      inputMessage = request->getParam("cmd")->value();

      if (inputMessage == "W") {
        x_ref += 30; // Positon ref if now -30 cm if we press Forward 
        start = true;
      }
      else if (inputMessage == "S") {
        x_ref -= 30; // Positon ref if now +30 cm if we press Backward
        start = true;
      }
      else if(inputMessage == "X") {
        reset = true; // Reset all quantities related to segway position to 0 
                      //and stop the segway
      }
      else if(inputMessage == "A") {
        turn_cmmd += 30; // Rotation of 30° to the left
        start = true;
      }
      else if(inputMessage == "D") {
        turn_cmmd -= 30; // Rotation of 30° to the right
        start = true;
      }
      else if(inputMessage == "1") {
        chosenPathCommands = chosenCommands[0];
        numCommandsChosen = *numCmd[0]; 
      }
      else if(inputMessage == "2") {
        chosenPathCommands = chosenCommands[1];
        numCommandsChosen = *numCmd[1];
      }
      else if(inputMessage == "3") {
        chosenPathCommands = chosenCommands[2];
        numCommandsChosen = *numCmd[2];
      }
      else if(inputMessage == "4") {
        chosenPathCommands = chosenCommands[3];
        numCommandsChosen = *numCmd[3];
      }
      else if(inputMessage == "5") {
        chosenPathCommands = chosenCommands[4];
        numCommandsChosen = *numCmd[4];
      }
    }
     request->send(200, "text/plain", "OK");
  });

  server.begin(); 

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
  t_start=micros();

  // check accelerometer 
  if (bno08x.wasReset()) { 
    Serial.print("sensor was reset "); 
    setReports(reportType, reportIntervalUs); 
  } 

  yaw = ypr.yaw - yaw_offset;

  // read encoders and convert to radians
  rad1 = -(encoder1.getCount()/4)*2*pi / (32*Rapport); 
  rad2 = (encoder2.getCount()/4)*2*pi / (32*Rapport); 

  //Position of the segway in [cm]
  pos_1 = rad1 * R;
  pos_2 = rad2 * R;
  pos = (pos_1 + pos_2)/2; //Average of the position of the 2 wheels
  yaw_wheels = (pos_1-pos_2)/L * 180/pi;

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


  if (numCommandsChosen > 2) {
    start = true;
    if (cntr_yaw == 2) { 
      funct_yaw_ref ( *(chosenPathCommands + cntr_yaw ) ); //[0] and [1] are ignored as they are initializations (cntr_yaw_pos starts at 2)
      cntr_yaw += 2;
    }
    if ((abs(turn_cmmd - yaw_wheels) < 2) && cntr_pos == 3) {
      funct_pos_ref ( *(chosenPathCommands + cntr_pos) );
      cntr_pos += 2;
    }
    if ((abs(x_ref - pos) < position_error) && (cntr_yaw < cntr_pos) && (cntr_yaw < numCommandsChosen)){ //When position error is small, the segway is at the correct position so it starts to turn in the direction of the next point
      funct_yaw_ref ( *(chosenPathCommands + cntr_yaw) );
      cntr_yaw += 2;
    }
    if ((abs(turn_cmmd - yaw_wheels) < 2) && (cntr_pos < cntr_yaw) && (cntr_pos < numCommandsChosen)){ //When yaw error is small, the segway is at the correct angle so it starts to move in the direction of the next point
      funct_pos_ref ( *(chosenPathCommands + cntr_pos) );
      cntr_pos += 2;
    }
    if ((cntr_yaw == numCommandsChosen) && (cntr_pos == numCommandsChosen+1) && (abs(x_ref - pos) < 5)){
      reset = true;
    }
  }

  if(start){
    K_P = K_P_move;
    K_D = K_D_move;
  }else{
    K_P = K_P_stable;
    K_D = K_D_stable;
  }

  if(reset){
    encoder1.setCount(0);
    encoder2.setCount(0);
    x_ref = 0;
    turn_cmmd = 0;
    rad1 = 0;
    rad2 = 0;
    pos_1 = 0;
    pos_2 = 0;
    pos = 0;
    yaw = 0;
    /*pitch_err=0;
    pre_error=0; 
    sum_error=0; 
    sum_p_error=0;
    sum_y_error=0;
    sum_error_speed = 0;
    previous_error_speed = 0;
    previous_error = 0;*/
    numCommandsChosen = 2;
    cntr_yaw = 2;
    cntr_pos = 3;
    K_P = K_P_stable;
    K_D = K_D_stable;
    start = false;

    x = 0;
    yaw_cmmd = 0;

    reset = false;
  }

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
    pitch = ypr.pitch - pitch_bias; // adjusting pitch with bias

    // Create an input spike when starting motion
    if(prevSpeed==0 && prevSpeed != refSpeed){
      x=D_Start(refSpeed, prevSpeed);
    }/*
    if(abs(t) <= position_error && !hasReset){
      x = Stop(D_stop, average_speed, prevSpeed);
      hasReset = true;
    }*/
    /*else if(abs(pos-x_ref)<position_error && average_speed>MinSpeed){
      x=D_stop(average_speed, prevSpeed);
    }*/
    else{
      x = PI_p_feedback(Kp_P, Kp_I, average_speed, refSpeed); // calculating reference angle based on reference speed
    }//Add desired speed 

    prevSpeed = average_speed; // updating previous speed variable

    pitch_err = pitch +x ; // adding reference angle to current angle with bias

    x_cmmd = PID_feedback(pitch_err, K_P, K_I, K_D); // Computes command to give to the motors (forwards/backwards motion only)
    yaw_cmmd = PI_y_feedback(Ky_P, Ky_I, turn_cmmd, yaw_wheels);

    avail_turn = (255 - abs(x_cmmd));

    if(abs(yaw_cmmd) > avail_turn) yaw_cmmd = sgn(yaw_cmmd) * avail_turn;

    Travel(x_cmmd, yaw_cmmd); // Function that instructs motors what to do

    // time management, making every loop iteration exactly 10ms
    t_end=micros();
    t_loop=t_end-t_start;
    while(t_loop > timeOverflow){
      timeOverflow+=100;
    }
    DeltaTime = timeOverflow/1000;
    delayMicroseconds(timeOverflow - t_loop);
  //}  
  t_end=micros();
}