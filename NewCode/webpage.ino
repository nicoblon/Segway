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
          <p>x Command: <span id="x_cmmdVal">Loading...</span></p>
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
            <button class="button" onclick="sendcmmd('1')">Path 0</button>
            <button class="button" onclick="sendcmmd('2')">Path 1</button>
            <button class="button" onclick="sendcmmd('3')">Path 2</button>
            <button class="button" onclick="sendcmmd('4')">Path 3</button>
            <button class="button" onclick="sendcmmd('5')">Path 4</button>
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
                document.getElementById("x_cmmdVal").innerHTML = obj.x_cmmd.toFixed(0);
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

  /*server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = "{\"yaw_wheels\":" + String(yaw_wheels) + 
                  ",\"ypr_yaw\":" + String(yaw) +
                  ",\"average_speed\":" + String(average_speed) +
                  ",\"position_error\":" + String(position_error) +
                  ",\"ypr_pitch\":" + String(ypr.pitch) + 
                  ",\"x_cmmd\":" + String(x_cmmd) + 
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
}