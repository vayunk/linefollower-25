#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h> // NEW: Library for persistent storage

/* * ESP32 SMART PID LINE FOLLOWER + WIFI TUNING
 * * Features:
 * 1. Persistent Storage: Kp, Kd, Speed, Delay saved to flash.
 * 2. Live Sensor/Motor Output: For detailed tuning feedback.
 * 3. Smart Search Logic: Coasts on straight-line loss, spins on corner loss.
 * 4. Error Memory: "Peak Hold" to ignore sensor noise on turns.
 */

// --- PIN DEFINITIONS ---
// Motors (TB6612FNG)
const int PWMA = 25;
const int AIN1 = 26;
const int AIN2 = 27;
const int PWMB = 14;
const int BIN1 = 12;
const int BIN2 = 13;
const int STBY = 4;

// Sensors (5 Channel)
const int S1 = 36; // Far Left
const int S2 = 39; // Left
const int S3 = 34; // Middle
const int S4 = 35; // Right
const int S5 = 32; // Far Right

// --- ROBOT STATE ---
bool isRunning = false;
bool blackLine = true; // true = Black line on White, false = White on Black

// --- TUNABLE PARAMETERS (Default values - overridden by Flash) ---
float Kp = 20.0;
float Ki = 0.0;
float Kd = 15.0;
int baseSpeed = 150; // PWM value (0-255)
int maxSpeed = 220;
int loopDelay = 5;   // Milliseconds to wait between updates (Stabilizer)
int errorMemory = 0; // NEW: Milliseconds to hold a strong error (Peak Hold)

// --- PID VARIABLES ---
int lastError = 0;
float P, I, D;
unsigned long lastProcessTime = 0;

// --- MEMORY VARIABLES ---
int retainedError = 0;
unsigned long lastStrongErrorTime = 0;

// --- LIVE OUTPUT VARIABLES ---
volatile int liveLeftSpeed = 0;
volatile int liveRightSpeed = 0;

// --- LOST LINE TRACKING ---
int lastKnownDirection = 0; // 1 = Right, -1 = Left

// --- PERSISTENCE OBJECT ---
Preferences preferences;


// --- FUNCTION DEFINITIONS FOR PERSISTENCE ---

void savePID() {
  preferences.begin("robot_config", false); // R/W mode
  preferences.putFloat("kp", Kp);
  preferences.putFloat("kd", Kd);
  preferences.putInt("speed", baseSpeed);
  preferences.putInt("delay", loopDelay);
  preferences.putInt("mem", errorMemory); // Save Memory setting
  preferences.end();
  Serial.println("Config Saved to Flash.");
}

void loadPID() {
  preferences.begin("robot_config", true); // Read-only mode
  
  // Retrieve saved values, using the default global values if not found.
  Kp = preferences.getFloat("kp", Kp);
  Kd = preferences.getFloat("kd", Kd);
  baseSpeed = preferences.getInt("speed", baseSpeed);
  loopDelay = preferences.getInt("delay", loopDelay);
  errorMemory = preferences.getInt("mem", errorMemory);
  
  preferences.end();
  Serial.println("Config Loaded from Flash.");
}


// --- WIFI SERVER ---
WebServer server(80);
const char* ssid = "LineFollower_Setup";
const char* password = "12345678";

// HTML Page Code (Stored as String)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Tuning</title>
  <style>
    body { font-family: Arial; text-align: center; margin:0; padding:20px; background-color: #222; color: white;}
    h2 { color: #ffcc00; }
    .slider { width: 80%; }
    .card { background-color: #333; padding: 20px; margin: 10px; border-radius: 10px; }
    button { padding: 15px 30px; font-size: 20px; border-radius: 5px; border: none; cursor: pointer; margin: 10px;}
    .btn-start { background-color: #4CAF50; color: white; }
    .btn-stop { background-color: #f44336; color: white; }
    .btn-save { background-color: #2196F3; color: white; font-weight: bold; } 
    .btn-toggle { background-color: #555; color: white; margin-top: 5px; padding: 10px; font-size: 16px;}
    .sensor-grid { display: flex; justify-content: center; margin-top: 15px; }
    .sensor-light { width: 30px; height: 30px; border-radius: 50%; margin: 0 5px; background-color: #555; border: 2px solid #999; }
    .sensor-active { background-color: #4CAF50 !important; border: 2px solid #fff; }
    .motor-output { margin-top: 15px; font-size: 1.2em; font-weight: bold; color: #ffcc00; }
  </style>
</head>
<body>
  <h2>ESP32 Line Follower</h2>
  <div class="card">
    <p>Status: <span id="status">%STATUS%</span></p>
    <button class="btn-start" onclick="sendCommand('start')">START</button>
    <button class="btn-stop" onclick="sendCommand('stop')">STOP</button>
    <button class="btn-save" onclick="sendCommand('save')">SAVE CONFIG</button>
    <br>
    <button class="btn-toggle" onclick="sendCommand('toggleLine')">Switch Line Color</button>
    <p>Current Mode: <span id="lineMode">%LINEMODE%</span></p>
  </div>

  <div class="card">
    <h3>Live Sensor Data</h3>
    <div class="sensor-grid">
      <div id="S1" class="sensor-light"></div>
      <div id="S2" class="sensor-light"></div>
      <div id="S3" class="sensor-light"></div>
      <div id="S4" class="sensor-light"></div>
      <div id="S5" class="sensor-light"></div>
    </div>
    <p>Error: <span id="currentError">--</span></p>
    <p>Last Error: <span id="lastError">--</span></p>
    <div class="motor-output">
      Left: <span id="motorL">---</span> | Right: <span id="motorR">---</span>
    </div>
  </div>

  <div class="card">
    <p>Kp: <span id="valKp">%KP%</span></p>
    <input type="range" min="0" max="100" step="0.1" value="%KP%" class="slider" onchange="updateVal('Kp', this.value)">
    
    <p>Kd: <span id="valKd">%KD%</span></p>
    <input type="range" min="0" max="100" step="0.1" value="%KD%" class="slider" onchange="updateVal('Kd', this.value)">
    
    <p>Speed: <span id="valSpeed">%SPEED%</span></p>
    <input type="range" min="0" max="255" value="%SPEED%" class="slider" onchange="updateVal('Speed', this.value)">

    <p>Loop Delay: <span id="valDelay">%DELAY%</span> ms</p>
    <input type="range" min="0" max="50" value="%DELAY%" class="slider" onchange="updateVal('Delay', this.value)">
    
    <p>Error Memory: <span id="valMem">%MEM%</span> ms</p>
    <input type="range" min="0" max="50" value="%MEM%" class="slider" onchange="updateVal('Mem', this.value)">
  </div>

<script>
const SENSOR_PINS = ["S1", "S2", "S3", "S4", "S5"];

function sendCommand(cmd) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/" + cmd, true);
  xhr.send();
  setTimeout(location.reload.bind(location), 500);
}

function updateVal(param, val) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/set?param=" + param + "&val=" + val, true);
  xhr.send();
  document.getElementById("val" + param).innerHTML = val;
}

function getSensorData() {
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      const data = JSON.parse(this.responseText);
      document.getElementById("currentError").innerHTML = data.error;
      document.getElementById("lastError").innerHTML = data.lasterror;
      document.getElementById("motorL").innerHTML = data.motorL;
      document.getElementById("motorR").innerHTML = data.motorR;

      // Update lights based on state
      for (let i = 0; i < SENSOR_PINS.length; i++) {
        const sensorPin = SENSOR_PINS[i];
        const sensorValue = data[sensorPin];
        const element = document.getElementById(sensorPin);

        if (sensorValue == 1) {
          element.classList.add('sensor-active');
        } else {
          element.classList.remove('sensor-active');
        }
      }
    }
  };
  xhr.open("GET", "/sensors", true);
  xhr.send();
}

// Start polling sensor data every 50ms
setInterval(getSensorData, 50); 
</script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);

  // LOAD CONFIGURATION FROM FLASH
  loadPID(); 
  
  // Setup Pins
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT);
  pinMode(S4, INPUT); pinMode(S5, INPUT);

  digitalWrite(STBY, HIGH); // Enable Motor Driver

  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Setup Web Server Routes
  server.on("/", handleRoot);
  server.on("/start", [](){ isRunning = true; server.send(200, "text/plain", "Started"); });
  server.on("/stop", [](){ isRunning = false; stopMotors(); server.send(200, "text/plain", "Stopped"); });
  
  // NEW: Dedicated Save Button Handler
  server.on("/save", [](){ 
    savePID(); 
    server.send(200, "text/plain", "Configuration Saved!"); 
  });

  server.on("/toggleLine", [](){ blackLine = !blackLine; server.send(200, "text/plain", "Toggled"); });
  server.on("/set", handleSetParams);
  server.on("/sensors", handleSensors); 
  
  server.begin();
    
  while (getError() != 0) {
      server.handleClient(); // Keep WiFi active so you can tune while waiting
      delay(50);             // Check 20 times a second
  }

  // Line found! Give user 1 second to pull hand away
  Serial.println("LINE FOUND! GOING IN 1 SECOND...");
  delay(1000); 
  isRunning = true;
}

void loop() {
  server.handleClient(); // Handle WiFi requests

  if (isRunning) {
    lineFollowLoop();
  } else {
    stopMotors();
  }
}

// --- MAIN LINE FOLLOWING LOGIC ---
void lineFollowLoop() {
  // LOOP PACING / DEBOUNCE
  if (millis() - lastProcessTime < loopDelay) {
    return; 
  }
  lastProcessTime = millis();

  int rawError = getError();
  
  // --- ERROR MEMORY (PEAK HOLD) ---
  // If the new error is stronger (larger magnitude) than what we are holding, update immediately.
  // This ensures fast reaction to entering a curve.
  if (abs(rawError) >= abs(retainedError) || rawError == 10) {
    retainedError = rawError;
    lastStrongErrorTime = millis();
  } 
  // If the new error is weaker (e.g. 0), hold the old strong error until timer expires.
  else {
    if (millis() - lastStrongErrorTime > errorMemory) {
      retainedError = rawError; // Memory expired, accept the weak error
    }
    // Else: Keep retainedError as it is (Memory active)
  }
  
  int error = retainedError;

  // CHECK 1: If line is found, update lastKnownDirection
  if (error != 10) { 
    if (abs(error) > 1) { 
        lastKnownDirection = (error > 0) ? 1 : -1; 
    }
  }

  // CHECK 2: If line is lost (Error 10) - SEARCH LOGIC
  if (error == 10) {
    // If the last position was CENTER (Error == 0), assume it's a gap and COAST straight.
    if (abs(lastError) <= 2) {
      liveLeftSpeed = baseSpeed;
      liveRightSpeed = baseSpeed;
      drive(liveLeftSpeed, liveRightSpeed);
      return; 
    }

    // INSTANT SPIN Search
    int direction = (lastKnownDirection >= 0) ? 1 : -1;
    if (direction == 1) {
      liveLeftSpeed = baseSpeed * 0.8; 
      liveRightSpeed = -baseSpeed * 0.8;
    } else {
      liveLeftSpeed = -baseSpeed * 0.8;
      liveRightSpeed = baseSpeed * 0.8;
    }
    drive(liveLeftSpeed, liveRightSpeed);
    return;
  }
  
  // CHECK 3: PID Calculation
  P = error;
  I = I + error; 
  D = error - lastError;
  lastError = error;

  float PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);

  int leftSpeed = baseSpeed + PIDvalue;
  int rightSpeed = baseSpeed - PIDvalue;

  liveLeftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  liveRightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  drive(liveLeftSpeed, liveRightSpeed);
}

// --- SENSOR READING ---
int getError() {
  bool s1 = blackLine ? !digitalRead(S1) : digitalRead(S1);
  bool s2 = blackLine ? !digitalRead(S2) : digitalRead(S2);
  bool s3 = blackLine ? !digitalRead(S3) : digitalRead(S3);
  bool s4 = blackLine ? !digitalRead(S4) : digitalRead(S4);
  bool s5 = blackLine ? !digitalRead(S5) : digitalRead(S5);

  // Single Sensors
  if (s1 && !s2 && !s3 && !s4 && !s5) return -4; // Far Left
  if (!s1 && s2 && !s3 && !s4 && !s5) return -2; // Left
  if (!s1 && !s2 && s3 && !s4 && !s5) return 0;  // Center
  if (!s1 && !s2 && !s3 && s4 && !s5) return 2;  // Right
  if (!s1 && !s2 && !s3 && !s4 && s5) return 4;  // Far Right

  // Two Sensors (Common Transitions)
  if (s1 && s2  && !s3 && !s4 && !s5) return -3; // Avg(-4, -2) = -3
  if (!s1 && s2 && s3  && !s4 && !s5) return -3; // Avg(-2, 0) = -1
  if (!s1 && !s2 && s3 && s4  && !s5) return 3;  // Avg(0, 2) = 1
  if (!s1 && !s2 && !s3 && s4 && s5)  return 3;  // Avg(2, 4) = 3

  // Three Sensors (Thick Lines / Intersections)
  if (s1 && s2 && s3 && !s4 && !s5) return -3;   // 11100 -> Avg(-4,-2,0) = -2
  if (!s1 && s2 && s3 && s4 && !s5) return 0;    // 01110 -> Avg(-2,0,2) = 0 (Center)
  if (!s1 && !s2 && s3 && s4 && s5) return 3;
  // Four Sensors (Thick Lines / Intersections)
  if (s1 && s2 && s3 && s4 && !s5) return -4;   // 11110 -> Avg(-4,-2,0) = -2
  if (!s1 && s2 && s3 && s4 && s5) return 4;    // 01111 -> Avg(0,2,4) = 2
  
  return 10; 
}

// --- MOTOR CONTROL ---
void drive(int left, int right) {
  if (left > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, abs(left));

  if (right > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, abs(right));
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

// --- WEB SERVER HANDLERS ---
void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");

  String s = index_html;
  s.replace("%STATUS%", isRunning ? "RUNNING" : "STOPPED");
  s.replace("%LINEMODE%", blackLine ? "Black Line" : "White Line");
  s.replace("%KP%", String(Kp));
  s.replace("%KD%", String(Kd));
  s.replace("%SPEED%", String(baseSpeed));
  s.replace("%DELAY%", String(loopDelay));
  s.replace("%MEM%", String(errorMemory));
  server.send(200, "text/html", s);
}

void handleSetParams() {
  if (server.hasArg("param") && server.hasArg("val")) {
    String param = server.arg("param");
    float val = server.arg("val").toFloat();

    if (param == "Kp") Kp = val;
    if (param == "Kd") Kd = val;
    if (param == "Speed") baseSpeed = (int)val;
    if (param == "Delay") loopDelay = (int)val;
    if (param == "Mem") errorMemory = (int)val;
    
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleSensors() {
  int rawS1 = digitalRead(S1);
  int rawS2 = digitalRead(S2);
  int rawS3 = digitalRead(S3);
  int rawS4 = digitalRead(S4);
  int rawS5 = digitalRead(S5);

  bool s1 = blackLine ? !rawS1 : rawS1;
  bool s2 = blackLine ? !rawS2 : rawS2;
  bool s3 = blackLine ? !rawS3 : rawS3;
  bool s4 = blackLine ? !rawS4 : rawS4;
  bool s5 = blackLine ? !rawS5 : rawS5;
  
  String json = "{\"S1\":" + String(s1) + 
                ",\"S2\":" + String(s2) + 
                ",\"S3\":" + String(s3) + 
                ",\"S4\":" + String(s4) + 
                ",\"S5\":" + String(s5) +
                ",\"error\":" + String(getError()) + 
                ",\"lasterror\":" + String(lastError) + 
                ",\"motorL\":" + String(liveLeftSpeed) +
                ",\"motorR\":" + String(liveRightSpeed) + "}";

  server.send(200, "application/json", json);
}