#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Access point credentials
const char* ssid = "ESP32-FaceTracker";
const char* password = "12345678";

// Web server on port 80
WebServer server(80);

// Servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVOMIN  150 // Minimum pulse length count
#define SERVOMAX  600 // Maximum pulse length count
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SG90MIN 0
#define SG90MAX 500

// Define serial port for COM8 monitoring
HardwareSerial SerialCOM8(2); // Using UART2

// Pins for COM8 serial connection
#define RX2_PIN 16  // GPIO16 as RX for COM8
#define TX2_PIN 17  // GPIO17 as TX for COM8

// Logging configuration
#define MAX_LOG_ENTRIES 50
String serialLogs[MAX_LOG_ENTRIES];
int logIndex = 0;

// Face coordinates
int x = 0;
int y = 0;
int z = 0;
bool faceDetected = false;

// Buffer for receiving coordinates
char buffer[16];
int bufferIndex = 0;

void setup() {
  // Start debug serial on default UART at 115200 baud
  Serial.begin(115200);
  Serial.println("\nESP32 Face Tracking and Servo Control");
  
  // Initialize I2C and Servo Driver
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Start COM8 connection on UART2 at 9600 baud
  SerialCOM8.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  
  // Configure ESP32 as an access point
  WiFi.softAP(ssid, password);
  
  // Get IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);
  
  // Define server routes
  server.on("/", handleRoot);
  server.on("/logs", handleLogs);
  server.on("/coordinates", handleCoordinates);
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Connect to WiFi SSID: " + String(ssid) + " with password: " + String(password));
  Serial.println("Then visit http://192.168.4.1 in your browser");
  
  // Add initial log
  addToLog("ESP32 Face Tracking Monitor started");
  addToLog("Listening on COM8 at 9600 baud (RX:" + String(RX2_PIN) + ", TX:" + String(TX2_PIN) + ")");
}

void loop() {
  // Handle client requests
  server.handleClient();
  
  // Check for data from COM8
  readFaceCoordinates();
  
  // Move servos if face is detected
  if (faceDetected) {
    moveServosToFace();
  }
}

// Read and process data from COM8
void readFaceCoordinates() {
  while (SerialCOM8.available() > 0) {
    char inChar = SerialCOM8.read();
    
    // Echo to debug serial
    Serial.write(inChar);
    
    // Process for face coordinates
    // Check if this is the start of a coordinate packet
    if (inChar == '(') {
      bufferIndex = 0;
      buffer[bufferIndex++] = inChar;
    }
    // Add character to buffer
    else if (bufferIndex > 0 && bufferIndex < 15) {
      buffer[bufferIndex++] = inChar;
      
      // Check if we have complete coordinates
      if (inChar == ')') {
        buffer[bufferIndex] = '\0'; // Null terminate
        parseCoordinates();
      }
    }
  }
}

// Parse coordinates from buffer in format "(xxx,yyy,zzz)"
void parseCoordinates() {
  // Make a copy of the buffer for logging
  String coordStr = String(buffer);
  
  // Check if the data format is correct
  if (buffer[0] == '(' && buffer[bufferIndex-1] == ')') {
    // Format should be "(xxx,yyy,zzz)"
    char xStr[4] = {0};
    char yStr[4] = {0};
    char zStr[4] = {0};
    
    // Extract X value (chars 1,2,3)
    int xIdx = 0;
    for (int i = 1; i <= 3 && i < bufferIndex; i++) {
      if (buffer[i] >= '0' && buffer[i] <= '9') {
        xStr[xIdx++] = buffer[i];
      }
    }
    xStr[xIdx] = '\0';
    
    // Extract Y value (chars 5,6,7)
    int yIdx = 0;
    for (int i = 5; i <= 7 && i < bufferIndex; i++) {
      if (buffer[i] >= '0' && buffer[i] <= '9') {
        yStr[yIdx++] = buffer[i];
      }
    }
    yStr[yIdx] = '\0';
    
    // Extract Z value (chars 9,10,11)
    int zIdx = 0;
    for (int i = 9; i <= 11 && i < bufferIndex; i++) {
      if (buffer[i] >= '0' && buffer[i] <= '9') {
        zStr[zIdx++] = buffer[i];
      }
    }
    zStr[zIdx] = '\0';
    
    // Convert to integers
    if (xIdx > 0 && yIdx > 0 && zIdx > 0) {
      int newX = atoi(xStr);
      int newY = atoi(yStr);
      int newZ = atoi(zStr);
      
      // Only log and update if coordinates change
      if (newX != x || newY != y || newZ != z || !faceDetected) {
        x = newX;
        y = newY;
        z = newZ;
        faceDetected = true;
        
        // Log the new coordinates
        addToLog("Face at " + coordStr + " (X:" + String(x) + ", Y:" + String(y) + ", Z:" + String(z) + ")");
      }
    }
  } else {
    // Invalid format
    addToLog("Invalid data: " + coordStr);
  }
  
  // Reset buffer
  bufferIndex = 0;
}

// Move servos based on face coordinates
void moveServosToFace() {
  // Map face coordinates to servo positions
  int xPos = map(x, 0, 999, SG90MIN, SG90MAX);
  int yPos = map(y, 0, 999, SG90MIN, SG90MAX);
  int zPos = map(z, 0, 999, SG90MIN, SG90MAX);
 
  // Move servos to follow face
  pwm.setPWM(0, 0, xPos); // X-axis servo
  pwm.setPWM(1, 0, yPos); // Y-axis servo
  pwm.setPWM(2, 0, zPos); // Z-axis servo
}

// Add message to log buffer
void addToLog(String message) {
  // Add timestamp
  unsigned long timeStamp = millis() / 1000;  // seconds since start
  String timeStr = String(timeStamp) + "s: ";
  
  // Add to log array
  serialLogs[logIndex] = timeStr + message;
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;  // Circular buffer
  
  // Also print to debug serial
  Serial.println(timeStr + message);
}

// Handle root URL
void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Face Tracking and Servo Control</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }";
  html += "h1 { color: #0066cc; }";
  html += "#logContainer { background-color: #1e1e1e; color: #dcdcdc; padding: 10px; border-radius: 5px; height: 30vh; overflow-y: scroll; margin-top: 20px; font-family: monospace; white-space: pre-wrap; }";
  html += ".faceCoordinates { background-color: #ffffff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; }";
  html += ".coordinate { display: inline-block; margin: 0 20px; text-align: center; }";
  html += ".value { font-size: 36px; font-weight: bold; color: #0066cc; }";
  html += ".label { font-size: 14px; color: #666; margin-top: 5px; }";
  html += ".status { font-size: 16px; margin-top: 15px; color: #333; }";
  html += ".face-status { font-weight: bold; }";
  html += ".face-detected { color: green; }";
  html += ".face-not-detected { color: red; }";
  html += ".refresh-note { font-size: 12px; color: #666; margin-top: 10px; }";
  html += ".servo-info { margin-top: 20px; background-color: #f9f9f9; padding: 10px; border-radius: 5px; }";
  html += "</style>";
  html += "<script>";
  html += "function updateData() {";
  html += "  updateCoordinates();";
  html += "  updateLogs();";
  html += "}";
  html += "function updateLogs() {";
  html += "  fetch('/logs')";
  html += "    .then(response => response.text())";
  html += "    .then(data => {";
  html += "      document.getElementById('logContainer').innerHTML = data;";
  html += "      document.getElementById('logContainer').scrollTop = document.getElementById('logContainer').scrollHeight;";
  html += "    });";
  html += "}";
  html += "function updateCoordinates() {";
  html += "  fetch('/coordinates')";
  html += "    .then(response => response.json())";
  html += "    .then(data => {";
  html += "      document.getElementById('x-value').innerText = data.x;";
  html += "      document.getElementById('y-value').innerText = data.y;";
  html += "      document.getElementById('z-value').innerText = data.z;";
  html += "      const statusElement = document.getElementById('face-status');";
  html += "      if (data.detected) {";
  html += "        statusElement.innerText = 'DETECTED';";
  html += "        statusElement.className = 'face-status face-detected';";
  html += "      } else {";
  html += "        statusElement.innerText = 'NOT DETECTED';";
  html += "        statusElement.className = 'face-status face-not-detected';";
  html += "      }";
  html += "    });";
  html += "}";
  html += "// Initial update and set interval";
  html += "updateData();";
  html += "setInterval(updateData, 500);";
  html += "</script>";
  html += "</head>";
  html += "<body>";
  html += "<h1>Face Tracking and Servo Control</h1>";
  html += "<div class='faceCoordinates'>";
  html += "<div class='coordinate'>";
  html += "<div class='value' id='x-value'>" + String(x) + "</div>";
  html += "<div class='label'>X-Coordinate</div>";
  html += "</div>";
  html += "<div class='coordinate'>";
  html += "<div class='value' id='y-value'>" + String(y) + "</div>";
  html += "<div class='label'>Y-Coordinate</div>";
  html += "</div>";
  html += "<div class='coordinate'>";
  html += "<div class='value' id='z-value'>" + String(z) + "</div>";
  html += "<div class='label'>Z-Coordinate</div>";
  html += "</div>";
  html += "<div class='status'>Status: <span id='face-status' class='face-status " + String(faceDetected ? "face-detected" : "face-not-detected") + "'>" + String(faceDetected ? "DETECTED" : "NOT DETECTED") + "</span></div>";
  html += "<div class='refresh-note'>Data refreshes automatically every 500ms</div>";
  html += "</div>";
  html += "<div class='servo-info'>";
  html += "<h2>Servo Control</h2>";
  html += "<p>Servos on channels 0-2 will track face coordinates:</p>";
  html += "<ul>";
  html += "<li>Channel 0: X-axis Servo</li>";
  html += "<li>Channel 1: Y-axis Servo</li>";
  html += "<li>Channel 2: Z-axis Servo</li>";
  html += "</ul>";
  html += "</div>";
  html += "<h2>Communication Log</h2>";
  html += "<div id='logContainer'></div>";
  html += "</body>";
  html += "</html>";
  
  server.send(200, "text/html", html);
}

// Handle logs request
void handleLogs() {
  String logs = "";
  
  // Start from the oldest entry
  int startIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
  for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
    int idx = (startIndex + i) % MAX_LOG_ENTRIES;
    if (serialLogs[idx].length() > 0) {
      logs += serialLogs[idx] + "<br>";
    }
  }
  
  server.send(200, "text/html", logs);
}

// Handle coordinates request
void handleCoordinates() {
  String json = "{";
  json += "\"x\":" + String(x) + ",";
  json += "\"y\":" + String(y) + ",";
  json += "\"z\":" + String(z) + ",";
  json += "\"detected\":" + String(faceDetected ? "true" : "false");
  json += "}";
  
  server.send(200, "application/json", json);
}