#include <Adafruit_BNO08x.h> 
#include <Arduino.h> 
#include <stdio.h>
#include <math.h>
#define BNO08X_RESET -1 

// Webserver librairies
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebSrv.h> //Try also <ESPAsyncWebSrv.h> if it doesn't work
  #include <ESP32Encoder.h>

//Encoders
  ESP32Encoder encoder1;
  ESP32Encoder encoder2;

// WebServer stuff
  AsyncWebServer server(80);
  // Replace with your network credentials
  // Check that the device used is also connected to the same wifi
  const char* ssid = "Hugh G Rection";  
  const char* password = "12345678";

//---CONSTANTS---
    //Rayon roues
    const int R = 3.2; //[cm]
    const float rayon = 18/2; //[cm] =  wheelbase/2

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
    const int DeltaTime=10;
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
    float pitch_bias = 3.3; //change depending on center of gravity
    float Kp_P = 100;//0.5;
    float Kp_I = 25;//0.1;
    float Ky_P = 0.5;//0.5;
    float Ky_I = 1.5;//1;
    float MaxSpeed=0;
    float prevMaxSpeed=0;
    float x=0;

    float LeftMotorAdjustment = 0.975;
    float RightMotorAdjustment = 1;

    float D_start = 1000;

  // Tracking of error 
    float pitch_err=2.25;
    float pre_error=0; 
    float sum_error=0; 
    float sum_p_error=0;
    float sum_y_error=0;

    float pos_ref = 0;
    float pitch = 0.0; 
    float pitch_ref = 0.0;
    float yaw_ref = 0;

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
    int l; //for the left wheel
    int r; //for the right wheel

    float pos;
    float yaw;

    // Boolean used to reset all the quantities giving informations 
    //on the segway relative position
    bool reset = false;

    // Counters 
    int counterugh = 0;
    int tempCount = 0;

    // Boolean used to check if the robot started its trajectory after the initial balancing phase
    bool start = false;

    // Path points
    const int numPoints1 = 8;
    const int numPoints2 = 12;
    const int numPoints3 = 11;
    const int numPoints4 = 16;
    const int numPoints5 = 8;

    // PID field
    const char* P_input_S = "KPS";
    const char* P_input_M = "KPM";
    const char* I_input = "KI";
    const char* D_input_S = "KDS";
    const char* D_input_M = "KDM";
    const char* PB_input = "PB";
    const char* KPpos_input = "KPp";
    const char* KIpos_input = "KIp";
    const char* KPyaw_input = "KPy";
    const char* KIyaw_input = "KIy";
    const char* MaxSpeed_input = "MaxS";
    const char* LeftMotorAdjustment_input = "LMot";
    const char* RightMotorAdjustment_input = "RMot";
    const char* D_start_input = "Ds";

    //Timing terms
    unsigned long t_start;
    unsigned long t_end;
    unsigned long t_loop;

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
  String Max_Speed_Val = String(MaxSpeed);
  String LeftMotorAdjustment_val = String(LeftMotorAdjustment);
  String RightMotorAdjustment_val = String(RightMotorAdjustment);
  String D_start_val = String(D_start);

// HTML root page
const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>PID web tuning</title>
      <style>
        .container {
          display: flex;
          justify-content: center;
          align-items: center;
          height: 10vh;
        }
        .button {
          font-size: 24px;
          padding: 16px;
          margin: 8px;
        }
      </style>
    </head>
    <body>
      <h2>PID web tuning</h2>
      
      <p><span id="textPropValS">Proportional gain for stabilization(current: %PPS%) </span>
      <input type="number" id="KPS" value="%PPS%" min="0" max="100" step="1">
      <button onclick="implement_P_S()">Submit</button></p>

      <p><span id="textPropValM">Proportional gain for motion(current: %PPM%) </span>
      <input type="number" id="KPM" value="%PPM%" min="0" max="100" step="1">
      <button onclick="implement_P_M()">Submit</button></p>
      
      <p><span id="textIntVal">Integral gain (current: %II%) </span>
      <input type="number" id="KI" value="%II%" min="0" max="10" step="0.1">
      <button onclick="implement_I()">Submit</button></p>
      
      <p><span id="textDerValS">Derivative gain for stabilization(current: %DDS%) </span>
      <input type="number" id="KDS" value="%DDS%" min="0" max="100" step="1">
      <button onclick="implement_D_S()">Submit</button></p>

      <p><span id="textDerValM">Derivative gain for motion(current: %DDM%) </span>
      <input type="number" id="KDM" value="%DDM%" min="0" max="100" step="1">
      <button onclick="implement_D_M()">Submit</button></p>
      
      <p><span id="textPitchBiasVal">Pitch Bias (current: %BB%) </span>
      <input type="number" id="PB" value="%BB%" min="-45" max="45" step="0.1">
      <button onclick="implement_B()">Submit</button></p>

      <p><span id="textKPpVal">Pos. Proportional gain (current: %KPp%) </span>
      <input type="number" id="KPp" value="%KPp%" min="0" max="100" step="0.01">
      <button onclick="implement_KpP()">Submit</button></p>

      <p><span id="textKIpVal">Pos. Integral gain (current: %KIp%) </span>
      <input type="number" id="KIp" value="%KIp%" min="0" max="10" step="0.01">
      <button onclick="implement_KpI()">Submit</button></p>

      <p><span id="textKPyVal">Yaw Proportional gain (current: %KPy%) </span>
      <input type="number" id="KPy" value="%KPy%" min="0" max="10" step="0.01">
      <button onclick="implement_KyP()">Submit</button></p>

      <p><span id="textKIyVal">Yaw Integral gain (current: %KIy%) </span>
      <input type="number" id="KIy" value="%KIy%" min="0" max="10" step="0.01">
      <button onclick="implement_KyI()">Submit</button></p>

      <p><span id="textMaxSpeedVal">Maximum Speed (current: %MaxSpeed%) </span>
      <input type="number" id="MaxS" value="%MaxSpeed%" min="-10" max="10" step="0.01">
      <button onclick="implement_MaxSpeed()">Submit</button></p>

      <p><span id="textLeftMotorAdjustmentVal">Left Motor Adjustment (current: %LeftMotorAdjustment%) </span>
      <input type="number" id="LMot" value="%LeftMotorAdjustment%" min="0" max="1" step="0.001">
      <button onclick="implement_LeftMotorAdjustment()">Submit</button></p>

      <p><span id="textRightMotorAdjustmentVal">Right Motor Adjustment (current: %RightMotorAdjustment%) </span>
      <input type="number" id="RMot" value="%RightMotorAdjustment%" min="0" max="1" step="0.001">
      <button onclick="implement_RightMotorAdjustment()">Submit</button></p>

      <p><span id="textD_startVal">D_start (current: %D_start%) </span>
      <input type="number" id="Ds" value="%D_start%" min="0" max="100" step="0.1">
      <button onclick="implement_D_start()">Submit</button></p>

      <div class="container">
        <button class="button" onclick="sendcmmd('1')">Path 0</button>
        <button class="button" onclick="sendcmmd('2')">Path 1</button>
      </div>

      <div class="container">
        <button class="button" onclick="sendcmmd('3')">Path 2</button>
        <button class="button" onclick="sendcmmd('4')">Path 3</button>
        <button class="button" onclick="sendcmmd('5')">Path 4</button>
      </div>

      <div class="container">
        <button class="button" onclick="sendcmmd('W')">Forward</button>
      </div>

      <div class="container">
        <button class="button" onclick="sendcmmd('A')">Left</button>
        <button class="button" onclick="sendcmmd('X')">Stop</button>
        <button class="button" onclick="sendcmmd('D')">Right</button>
      </div>

      <div class="container">
        <button class="button" onclick="sendcmmd('S')">Backward</button>
      </div>

    <script>

      function implement_P_S(){
        var P_val_S = document.getElementById("KPS").value;
        document.getElementById("textPropValS").innerHTML = "Proportional gain for stabilization(current: " + P_val_S + ") ";
        console.log(P_val_S);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/proportionalS?KPS="+P_val_S, true);
        xhr.send();
      }

      function implement_P_M(){
        var P_val_M = document.getElementById("KPM").value;
        document.getElementById("textPropValM").innerHTML = "Proportional gain for motion(current: " + P_val_M + ") ";
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
        document.getElementById("textDerValS").innerHTML = "Derivative gain for stabilization(current: " + D_val_S + ") ";
        console.log(D_val_S);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/derivativeS?KDS="+D_val_S, true);
        xhr.send();
      }

      function implement_D_M(){
        var D_val_M = document.getElementById("KDM").value;
        document.getElementById("textDerValM").innerHTML = "Derivative gain for motion(current: " + D_val_M + ") ";
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
        document.getElementById("textKPpVal").innerHTML = "Pos. Proportional gain (current: " + Kp_P_val + ") ";
        console.log(Kp_P_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kppos?ET="+Kp_P_val, true);
        xhr.send();
      }

      function implement_KpI(){
        var Kp_I_val = document.getElementById("KIp").value;
        document.getElementById("textKIpVal").innerHTML = "Pos. Integral gain (current: " + Kp_I_val + ") ";
        console.log(Kp_I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kipos?ET="+Kp_I_val, true);
        xhr.send();
      }

      function implement_KyP(){
        var Ky_P_val = document.getElementById("KPy").value;
        document.getElementById("textKPyVal").innerHTML = "Yaw Proportional gain (current: " + Ky_P_val + ") ";
        console.log(Ky_P_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kpyaw?ET="+Ky_P_val, true);
        xhr.send();
      }

      function implement_KyI(){
        var Ky_I_val = document.getElementById("KIy").value;
        document.getElementById("textKIyVal").innerHTML = "Yaw Integral gain (current: " + Ky_I_val + ") ";
        console.log(Ky_I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kiyaw?ET="+Ky_I_val, true);
        xhr.send();
      }

      function implement_MaxSpeed(){
          var Max_Speed_Val = document.getElementById("MaxS").value;
          document.getElementById("textMaxSpeedVal").innerHTML = "Max Speed value (current: " + Max_Speed_Val + ") ";
          console.log(Max_Speed_Val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/maxspeed?MaxS="+Max_Speed_Val, true);
          xhr.send();
      }

      function implement_LeftMotorAdjustment(){
          var LeftMotorAdjustment_val = document.getElementById("LMot").value;
          document.getElementById("textLeftMotorAdjustmentVal").innerHTML = "Left Motor Adjustment Percentage (current: " + LeftMotorAdjustment_val + ") ";
          console.log(LeftMotorAdjustment_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/leftmotoradjustment?LMot="+LeftMotorAdjustment_val, true);
          xhr.send();
      }

      function implement_RightMotorAdjustment(){
          var RightMotorAdjustment_val = document.getElementById("RMot").value;
          document.getElementById("textRightMotorAdjustmentVal").innerHTML = "Right Motor Adjustment Percentage (current: " + RightMotorAdjustment_val + ") ";
          console.log(RightMotorAdjustment_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/rightmotoradjustment?RMot="+RightMotorAdjustment_val, true);
          xhr.send();
      }

      function implement_D_start(){
          var D_start_val = document.getElementById("Ds").value;
          document.getElementById("textD_startVal").innerHTML = "D_start Value (current: " + D_start_val + ") ";
          console.log(D_start_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/d_start?Ds="+D_start_val, true);
          xhr.send();
      }

      function sendcmmd(cmmd){
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/cmmd?cmd=" + cmmd, true);
            xhr.send();
      }

    </script>
    </body>
  </html>
)rawliteral";

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
  }else if (var == "KPy"){
    return Ky_P_val;
  }else if (var == "KIy"){
    return Ky_I_val;
  }else if (var == "MaxSpeed"){
    return Max_Speed_Val;
  }else if (var == "LeftMotorAdjustment"){
    return LeftMotorAdjustment_val;
  }else if (var == "RightMotorAdjustment"){
    return RightMotorAdjustment_val;
  }else if (var == "D_start"){
    return D_start_val;
  }
  return String();
}

// Accelerometer variables
  Adafruit_BNO08x  bno08x(BNO08X_RESET); 
  sh2_SensorValue_t sensorValue; 

struct euler_t { 
  float yaw; 
  float pitch; 
  float roll; 
} ypr; 
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

void setReports(sh2_SensorId_t reportType, long report_interval) { 
  Serial.println("Setting desired reports"); 
  if (! bno08x.enableReport(reportType, report_interval)) { 
    Serial.println("Could not enable stabilized remote vector"); 
  } 
} 
// Functions 
//basic sign function, it returns 1 if x is positive, and -1 if not. 
int sgn(int x){ 
  if (x >= 0){ 
    return 1; 
  }else{ 
    return -1; 
  } 
} 

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = true) { 

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

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) { 
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees); 
} 

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) { 
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees); 
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
  if (start){ //check whether the robot is moving or not to activate/deactivate the integral term 
    K_int=0;
  }
  else {
    K_int=K_I*sum_error; //integral part of the controller 
  }
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
// Function used to turn
void Travel(int x_command, int turn_command){// instruction to the motors with the command
  if(x_command>=0){// forwards and left, forwards more than left wheel
    analogWrite(M2A, LeftMotorAdjustment*x_command);
    analogWrite(M2B, 0);
    analogWrite(M1A, 0);
    analogWrite(M1B, RightMotorAdjustment*x_command);
  }
  else{ 
    analogWrite(M2A, 0);
    analogWrite(M2B, -LeftMotorAdjustment*x_command);
    analogWrite(M1A, -RightMotorAdjustment*x_command);
    analogWrite(M1B, 0);  
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

  server.on("/maxspeed", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET input1 value on <ESP_IP>/proportional?KP=<inputMessage>
    if (request->hasParam(MaxSpeed_input)) {
      inputMessage = request->getParam(MaxSpeed_input)->value();
      Max_Speed_Val = inputMessage;
      MaxSpeed = Max_Speed_Val.toFloat();
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

  server.begin(); 
  K_P = K_P_stable; 
  K_D = K_D_stable;

}

void loop() {
  // put your main code here, to run repeatedly:
  t_start=micros();
  if (bno08x.wasReset()) { 
    Serial.print("sensor was reset "); 
    setReports(reportType, reportIntervalUs); 
  } 

  rad1 = -(encoder1.getCount()/4)*2*pi / (32*Rapport); 
  rad2 = (encoder2.getCount()/4)*2*pi / (32*Rapport); 

  //Position of the segway in [cm]
  pos_1 = rad1 * R;
  pos_2 = rad2 * R;
  pos = (pos_1 + pos_2)/2; //Average of the position of the 2 wheels
  speed= (pos-prev_pos)/DeltaTime;
  updateSpeedBuffer(speed); //adds value to speedBuffer list
  average_speed=averageNonZero(speedBuffer, BUFFER_SIZE); //Calcul de la vitesse moyenne sur au plus 10 valeurs
  prev_pos=pos;

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

  // if (start){
  //   K_P = K_P_move; 
  //   K_D = K_D_move;
  //   pitch_ref = PI_p_feedback(Kp_P, Kp_I, pos, pos_ref);
  //   yaw_cmmd = -PI_y_feedback(Ky_P, Ky_I, yaw, yaw_ref);
  // }
  // else {
  // }
  if(MaxSpeed==0){
  K_P = K_P_stable; 
  K_D = K_D_stable;}
  else{
    K_P= K_P_move; 
    K_D=K_D_move;
  }
  pitch= ypr.pitch - pitch_bias; // Get the pitch angle. Minus comes from the change of wiring with the motor
    // This is were we include the pitch bias.
  if(prevMaxSpeed==0 && prevMaxSpeed != MaxSpeed){
    x=D_Start(MaxSpeed, prevMaxSpeed);
   }
  else{
    x = PI_p_feedback(Kp_P, Kp_I, average_speed, MaxSpeed);
  }//Add desired speed 
  prevMaxSpeed=MaxSpeed;
  pitch_err = pitch +x ; //add +x

  x_cmmd = PID_feedback(pitch_err, K_P, K_I, K_D); // Computes command to keep stable

  
  Travel(x_cmmd, 0); // Function that instructs motors what to do


  Serial.print("KP: ");
  Serial.println(K_P_move);
  Serial.print("KD: ");
  Serial.println(K_D_move);


  t_end=micros();
  t_loop=t_end-t_start;
  delayMicroseconds(10000-t_loop);
}
