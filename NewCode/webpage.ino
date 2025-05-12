#include "config.h"

AsyncWebServer server(80);
const char* ssid = "IPhone de Nicolas";
const char* password = "nicolebg";

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
          background-color: #f7f7f7;
          margin: 20px;
        }
        h2 {
          text-align: center;
        }
        .param {
          display: flex;
          align-items: center;
          justify-content: space-between;
          background: #ffffff;
          padding: 10px;
          margin: 10px 0;
          border-radius: 8px;
          box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
        }
        .param span {
          flex: 2;
          font-size: 14px;
          padding-right: 10px;
        }
        .param input {
          flex: 1;
          padding: 5px;
          font-size: 14px;
          border: 1px solid #ccc;
          border-radius: 4px;
          margin-right: 10px;
        }
        .param button {
          background-color: #ff69b4;
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
          padding: 12px;
          margin: 6px;
          border-radius: 8px;
          background-color: #4CAF50;
          color: white;
          border: none;
          cursor: pointer;
        }
      </style>
    </head>
    <body>
      <h2>PID web tuning</h2>

      <h3>Live Data</h3>
      <p>Yaw (from wheels): <span id="yawWheelsVal">Loading...</span> deg</p>
      
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

      <p><span id="textKPyVal">Yaw Proportional gain (current: %Ky_P%) </span>
      <input type="number" id="KPY" value="%Ky_P%" min="0" max="10" step="0.01">
      <button onclick="implement_KyP()">Submit</button></p>

      <p><span id="textKIyVal">Yaw Integral gain (current: %Ky_I%) </span>
      <input type="number" id="KIY" value="%Ky_I%" min="0" max="10" step="0.01">
      <button onclick="implement_KyI()">Submit</button></p>

      <p><span id="textLeftMotorAdjustmentVal">Left Motor Adjustment (current: %LeftMotorAdjustment%) </span>
      <input type="number" id="LMot" value="%LeftMotorAdjustment%" min="0" max="1" step="0.001">
      <button onclick="implement_LeftMotorAdjustment()">Submit</button></p>

      <p><span id="textRightMotorAdjustmentVal">Right Motor Adjustment (current: %RightMotorAdjustment%) </span>
      <input type="number" id="RMot" value="%RightMotorAdjustment%" min="0" max="1" step="0.001">
      <button onclick="implement_RightMotorAdjustment()">Submit</button></p>

      <p><span id="textD_startVal">D_start (current: %D_start%) </span>
      <input type="number" id="Ds" value="%D_start%" min="0" max="100" step="0.1">
      <button onclick="implement_D_start()">Submit</button></p>

      <p><span id="textD_stopVal">D_stop (current: %D_stop%) </span>
      <input type="number" id="Dsp" value="%D_stop%" min="0" max="10000" step="1">
      <button onclick="implement_D_stop()">Submit</button></p>

      <p><span id="textx_refVal">Reference position (current: %x_ref%) </span>
      <input type="number" id="Xref" value="%x_ref%" min="-10000" max="10000" step="1">
      <button onclick="implement_x_ref()">Submit</button></p>

      <p><span id="textKp_speedVal">Speed Proportional Gain (current: %Kp_speed%) </span>
      <input type="number" id="KP_sp" value="%Kp_speed%" min="0" max="10000" step="1">
      <button onclick="implement_Kp_speed()">Submit</button></p>

      <p><span id="textKi_speedVal">Speed Integral Gain (current: %Ki_speed%) </span>
      <input type="number" id="KI_sp" value="%Ki_speed%" min="0" max="10000" step="1">
      <button onclick="implement_Ki_speed()">Submit</button></p>

      <p><span id="textspeed_errVal">Speed Error (current: %speed_err%) </span>
      <input type="number" id="Sp_err" value="%speed_err%" min="0" max="10" step="0.01">
      <button onclick="implement_speed_err()">Submit</button></p>

      <p><span id="textposition_errorVal">Position Error (current: %position_error%) </span>
      <input type="number" id="pos_err" value="%position_error%" min="0" max="100" step="1">
      <button onclick="implement_position_error()">Submit</button></p>

      <p><span id ="textturn_cmmdVal">Turn Command (current: %turn_cmmd%) </span>
       <input type="number" id="t_c" value="%turn_cmmd%" min="-180" max="180" step="1">
       <button onclick="implement_turn_cmmd()">Submit</button></p>

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
        var Ky_P_val = document.getElementById("KPY").value;
        document.getElementById("textKPyVal").innerHTML = "Yaw Proportional gain (current: " + Ky_P_val + ") ";
        console.log(Ky_P_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kpyaw?KPY="+Ky_P_val, true);
        xhr.send();
      }

      function implement_KyI(){
        var Ky_I_val = document.getElementById("KIY").value;
        document.getElementById("textKIyVal").innerHTML = "Yaw Integral gain (current: " + Ky_I_val + ") ";
        console.log(Ky_I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kiyaw?KIY="+Ky_I_val, true);
        xhr.send();
      }

      function implement_x_ref(){
          var x_ref_val = document.getElementById("Xref").value;
          document.getElementById("textx_refVal").innerHTML = "Reference position (current: " + x_ref_val + ") ";
          console.log(x_ref_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/xref?Xref="+x_ref_val, true);
          xhr.send();
      }

      function implement_Kp_speed(){
          var Kp_speed_val = document.getElementById("KP_sp").value;
          document.getElementById("textKp_speedVal").innerHTML = "Speed Proportional Gain (current: " + Kp_speed_val + ") ";
          console.log(Kp_speed_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/kpspeed?KP_sp="+Kp_speed_val, true);
          xhr.send();
      }

      function implement_Ki_speed(){
          var Ki_speed_val = document.getElementById("KI_sp").value;
          document.getElementById("textKi_speedVal").innerHTML = "Speed Integral Gain (current: " + Ki_speed_val + ") ";
          console.log(Ki_speed_val);
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/kispeed?KI_sp="+Ki_speed_val, true);
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

      function implement_D_stop(){
          var D_stop_val = document.getElementById("Dsp").value;
          document.getElementById("textD_stopVal").innerHTML = "D_stop Value (current: " + D_stop_val + ") ";
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

      function updateData(){
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/data", true);
        xhr.onreadystatechange = function(){
            if(xhr.readyState == 4 && xhr.status == 200){
                var obj = JSON.parse(xhr.responseText);
                document.getElementById("yawWheelsVal").innerHTML = obj.yaw_wheels.toFixed(2);
            }
        };
        xhr.send();
      }

      // Refresh every 200 milliseconds (adjust if needed)
      setInterval(updateData, 200);


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
  }return String();
}

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

void serverStuff(void){
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

   server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"yaw_wheels\":" + String(yaw_wheels) + "}";
    request->send(200, "application/json", json);
  }); 


  server.begin(); 
}