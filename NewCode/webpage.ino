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

        <div class="path-buttons">
          <strong>Path Commands</strong><br>
          <button class="button" onclick="sendcmmd('1')">Path 1</button>
          <button class="button" onclick="sendcmmd('2')">Path 2</button>
          <button class="button" onclick="sendcmmd('3')">Path 3</button>
          <button class="button" onclick="sendcmmd('4')">Path 4</button>
          <button class="button" onclick="sendcmmd('5')">Circuit</button>
          <button class="button" onclick="sendcmmd('6')">Circuit2</button>
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

      <p><span id="textPitchBiasVal">Pitch Bias (current: %BB%) </span>
      <input type="number" id="PB" value="%BB%" min="-45" max="45" step="0.1"> 
      <button onclick="implement_B()">Submit</button></p>

      <p><span id="textLeftMotorAdjustmentVal">L Motor Adj. (current: %LeftMotorAdjustment%) </span>
      <input type="number" id="LMot" value="%LeftMotorAdjustment%" min="0" max="1" step="0.01"> 
      <button onclick="implement_LeftMotorAdjustment()">Submit</button></p>

      <p><span id="textRightMotorAdjustmentVal">R Motor Adj. (current: %RightMotorAdjustment%) </span>
      <input type="number" id="RMot" value="%RightMotorAdjustment%" min="0" max="1" step="0.01"> 
      <button onclick="implement_RightMotorAdjustment()">Submit</button></p>

      <p><span id="textKp_speedVal">Speed Prop. Gain (current: %Kp_speed%) </span>
      <input type="number" id="KP_sp" value="%Kp_speed%" min="0" max="10000" step="1"> 
      <button onclick="implement_Kp_speed()">Submit</button></p>

      <p><span id="textKi_speedVal">Speed Integral Gain (current: %Ki_speed%) </span>
      <input type="number" id="KI_sp" value="%Ki_speed%" min="0" max="10000" step="1"> 
      <button onclick="implement_Ki_speed()">Submit</button></p>

      <p><span id="textKPyVal">Yaw Prop. Gain (current: %Ky_P%) </span>
      <input type="number" id="KPY" value="%Ky_P%" min="0" max="100" step="1"> 
      <button onclick="implement_KyP()">Submit</button></p>

      <p><span id="textKIyVal">Yaw Integral Gain (current: %Ky_I%) </span>
      <input type="number" id="KIY" value="%Ky_I%" min="0" max="10" step="0.05">
        <button onclick="implement_KyI()">Submit</button></p>

    <script>

      function sendcmmd(cmmd){
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/cmmd?cmd=" + cmmd, true);
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

      function implement_B(){
        var PitchBias_val = document.getElementById("PB").value;
        document.getElementById("textPitchBiasVal").innerHTML = "Pitch Bias (current: " + PitchBias_val + ") ";
        console.log(PitchBias_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/pitchbias?PB="+PitchBias_val, true);
        xhr.send();
      }

      function implement_Kp_speed(){
          var Kp_speed_val = document.getElementById("KP_sp").value;
          document.getElementById("textKp_speedVal").innerHTML = "Speed Prop. Gain (current: " + Kp_speed_val + ") ";
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

      function implement_KyP(){
        var Ky_P_val = document.getElementById("KPY").value;
        document.getElementById("textKPyVal").innerHTML = "Yaw Prop. Gain (current: " + Ky_P_val + ") ";
        console.log(Ky_P_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kpyaw?KPY="+Ky_P_val, true);
        xhr.send();
      }

      function implement_KyI(){
        var Ky_I_val = document.getElementById("KIY").value;
        document.getElementById("textKIyVal").innerHTML = "Yaw Integral Gain (current: " + Ky_I_val + ") ";
        console.log(Ky_I_val);
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/kiyaw?KIY="+Ky_I_val, true);
        xhr.send();
      }

    </script>
    </body>
  </html>
)rawliteral";

// Replaces placeholder with button section in your web page
String processor(const String& var){
  if (var == "BB"){
    return PitchBias_val;
  }else if (var == "LeftMotorAdjustment"){
    return LeftMotorAdjustment_val;
  }else if (var == "RightMotorAdjustment"){
    return RightMotorAdjustment_val;
  }else if (var == "Kp_speed"){
    return Kp_speed_val;
  }else if (var == "Ki_speed"){
    return Ki_speed_val;
  }else if (var == "Ky_P"){
    return Ky_P_val;
  }else if (var == "Ky_I"){
    return Ky_I_val;
  }
  return String();
}

String PitchBias_val = String(pitch_bias);
String LeftMotorAdjustment_val = String(LeftMotorAdjustment);
String RightMotorAdjustment_val = String(RightMotorAdjustment);
String Kp_speed_val = String(Kp_speed);
String Ki_speed_val = String(Ki_speed);
String Ky_P_val = String(Ky_P);
String Ky_I_val = String(Ky_I);

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

  server.on("/cmmd", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("cmd")) {
      inputMessage = request->getParam("cmd")->value();

      if (inputMessage == "W") {
        x_ref += 100; // Positon ref if now -30 cm if we press Forward 
        start = true;
      }
      else if (inputMessage == "S") {
        x_ref -= 100; // Positon ref if now +30 cm if we press Backward
        start = true;
      }
      else if(inputMessage == "X") {
        reset = true; // Reset all quantities related to segway position to 0 
                      //and stop the segway
      }
      else if(inputMessage == "A") {
        turn_cmmd += 45; // Rotation of 30° to the left
        start = true;
      }
      else if(inputMessage == "D") {
        turn_cmmd -= 45; // Rotation of 30° to the right
        start = true;
      }
      else if(inputMessage == "1") {
        chosenPathCommands = chosenCommands[0];
        numCommandsChosen = *numCmd[0]; 
        circuit = false;
      }
      else if(inputMessage == "2") {
        chosenPathCommands = chosenCommands[1];
        numCommandsChosen = *numCmd[1];
        circuit = false;
      }
      else if(inputMessage == "3") {
        chosenPathCommands = chosenCommands[2];
        numCommandsChosen = *numCmd[2];
        circuit = false;
      }
      else if(inputMessage == "4") {
        chosenPathCommands = chosenCommands[3];
        numCommandsChosen = *numCmd[3];
        circuit = false;
      }
      else if(inputMessage == "5") {
        chosenPathCommands = chosenCommands[4];
        numCommandsChosen = *numCmd[4];
        circuit = true;
      }
      else if(inputMessage == "6") {
        chosenPathCommands = chosenCommands[5];
        numCommandsChosen = *numCmd[5];
        circuit = true;
      }
    }
     request->send(200, "text/plain", "OK");
  });

  server.begin(); 
}