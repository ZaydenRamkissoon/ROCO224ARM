#include <WiFi.h>
#include <WebServer.h>

// Access point credentials
const char* ssid = "ESP32-AP";  // Name of your access point
const char* password = "12345678";  // Password (must be at least 8 characters)

// Web server on port 80
WebServer server(80);

// Buffer to store serial output
#define MAX_LOG_ENTRIES 100
String serialLogs[MAX_LOG_ENTRIES];
int logIndex = 0;
unsigned long lastSerialCheck = 0;

// Face coordinates
int x = 0;
int y = 0;
int z = 0;
bool faceDetected = false;

// Buffer for receiving coordinates
char buffer[16];
int bufferIndex = 0;

void setup() {
  // Start serial communication
  Serial.begin(115200);
  Serial.println("\nESP32 Face Coordinates Monitor");
  
  // Configure ESP32 as an access point
  WiFi.softAP(ssid, password);
  
  // Get IP address
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP address: ");
  Serial.println(IP);
  
  // Define server routes
  server.on("/", handleRoot);
  server.on("/logs", handleLogs);
  server.on("/clear", handleClear);
  server.on("/coordinates", handleCoordinates);
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Connect to WiFi SSID: " + String(ssid) + " with password: " + String(password));
  Serial.println("Then visit http://192.168.4.1 in your browser");
  
  // Add initial logs
  addToSerialLog("ESP32 Face Coordinates Monitor started");
  addToSerialLog("Access Point IP: " + IP.toString());
}

void loop() {
  // Handle client requests
  server.handleClient();
  
  // Check for serial input periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialCheck > 100) {  // Check every 100ms
    lastSerialCheck = currentMillis;
    
    // Read from serial
    while (Serial.available() > 0) {
      char inChar = Serial.read();
      
      // Process for face coordinates
      processSerialByte(inChar);
      
      // Also add raw serial data to logs
      if (inChar == '\n') {
        String line = "";
        for (int i = 0; i < bufferIndex && i < 15; i++) {
          line += buffer[i];
        }
        if (line.length() > 0) {
          addToSerialLog("Serial: " + line);
        }
        bufferIndex = 0;
      } else if (bufferIndex < 15) {
        buffer[bufferIndex++] = inChar;
      }
    }
  }
}

// Process incoming serial byte for coordinate detection
void processSerialByte(char inChar) {
  static char coordBuffer[16];
  static int coordIndex = 0;
  
  // Start of coordinate packet
  if (inChar == '(') {
    coordIndex = 0;
    coordBuffer[coordIndex++] = inChar;
  }
  // Add character to buffer
  else if (coordIndex > 0 && coordIndex < 15) {
    coordBuffer[coordIndex++] = inChar;
    
    // Check if we have complete coordinates
    if (inChar == ')') {
      coordBuffer[coordIndex] = '\0'; // Null terminate
      parseCoordinates(coordBuffer, coordIndex);
    }
  }
}

// Parse coordinates from buffer in format "(xxx,yyy,zzz)"
void parseCoordinates(char* data, int length) {
  // Check if the data format is correct
  if (data[0] == '(' && data[length-1] == ')') {
    // Format should be "(xxx,yyy,zzz)"
    char xStr[4] = {0};
    char yStr[4] = {0};
    char zStr[4] = {0};
    
    // Extract X value (chars 1,2,3)
    int xIdx = 0;
    for (int i = 1; i <= 3 && i < length; i++) {
      if (data[i] >= '0' && data[i] <= '9') {
        xStr[xIdx++] = data[i];
      }
    }
    xStr[xIdx] = '\0';
    
    // Extract Y value (chars 5,6,7)
    int yIdx = 0;
    for (int i = 5; i <= 7 && i < length; i++) {
      if (data[i] >= '0' && data[i] <= '9') {
        yStr[yIdx++] = data[i];
      }
    }
    yStr[yIdx] = '\0';
    
    // Extract Z value (chars 9,10,11)
    int zIdx = 0;
    for (int i = 9; i <= 11 && i < length; i++) {
      if (data[i] >= '0' && data[i] <= '9') {
        zStr[zIdx++] = data[i];
      }
    }
    zStr[zIdx] = '\0';
    
    // Convert to integers
    if (xIdx > 0 && yIdx > 0 && zIdx > 0) {
      x = atoi(xStr);
      y = atoi(yStr);
      z = atoi(zStr);
      faceDetected = true;
      
      // Log the coordinates
      String coordMsg = "Face detected at X:" + String(x) + ", Y:" + String(y) + ", Z:" + String(z);
      addToSerialLog(coordMsg);
    }
  }
}

// Add message to serial log buffer
void addToSerialLog(String message) {
  // Add timestamp
  unsigned long timeStamp = millis() / 1000;  // seconds since start
  String timeStr = String(timeStamp) + "s: ";
  
  // Add to log array
  serialLogs[logIndex] = timeStr + message;
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;  // Circular buffer
}

// Handle root URL
void handleRoot() {
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32 Face Coordinates Monitor</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }";
  html += "h1 { color: #0066cc; }";
  html += "#logContainer { background-color: #1e1e1e; color: #dcdcdc; padding: 10px; border-radius: 5px; height: 40vh; overflow-y: scroll; margin-bottom: 10px; font-family: monospace; white-space: pre-wrap; }";
  html += ".controls { margin: 10px 0; }";
  html += "button { background-color: #4CAF50; border: none; color: white; padding: 10px 15px; text-align: center; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 5px; }";
  html += "#clearBtn { background-color: #f44336; }";
  html += ".faceCoordinates { background-color: #ffffff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; }";
  html += ".coordinate { display: inline-block; margin: 0 10px; text-align: center; }";
  html += ".value { font-size: 24px; font-weight: bold; color: #0066cc; }";
  html += ".label { font-size: 12px; color: #666; }";
  html += ".status { font-size: 14px; margin-top: 5px; color: #333; }";
  html += ".face-status { font-weight: bold; }";
  html += ".face-detected { color: green; }";
  html += ".face-not-detected { color: red; }";
  html += "</style>";
  html += "<script>";
  html += "function updateLogs() {";
  html += "  fetch('/logs')";
  html += "    .then(response => response.text())";
  html += "    .then(data => {";
  html += "      if(data.trim() !== '') {";
  html += "        const container = document.getElementById('logContainer');";
  html += "        container.innerHTML = data;";
  html += "        container.scrollTop = container.scrollHeight;";
  html += "      }";
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
  html += "function clearLogs() {";
  html += "  fetch('/clear')";
  html += "    .then(() => {";
  html += "      document.getElementById('logContainer').innerHTML = '';";
  html += "    });";
  html += "}";
  html += "// Initial update and set interval";
  html += "updateLogs();";
  html += "updateCoordinates();";
  html += "setInterval(updateLogs, 500);";
  html += "setInterval(updateCoordinates, 200);";
  html += "</script>";
  html += "</head>";
  html += "<body>";
  html += "<h1>Face Coordinates Monitor</h1>";
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
  html += "<div class='status'>Face Status: <span id='face-status' class='face-status " + String(faceDetected ? "face-detected" : "face-not-detected") + "'>" + String(faceDetected ? "DETECTED" : "NOT DETECTED") + "</span></div>";
  html += "</div>";
  html += "<div class='controls'>";
  html += "<button onclick='updateLogs()'>Refresh Logs</button>";
  html += "<button id='clearBtn' onclick='clearLogs()'>Clear Logs</button>";
  html += "</div>";
  html += "<h2>Serial Log</h2>";
  html += "<div id='logContainer'></div>";
  html += "<div class='status'>";
  html += "Connected to Access Point: " + String(ssid);
  html += "</div>";
  html += "</body>";
  html += "</html>";
  
  server.send(200, "text/html", html);
}

// Handle logs request
void handleLogs() {
  String logs = "";
  
  // Start from the oldest entry (assuming buffer is full)
  int startIndex = logIndex;
  for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
    int idx = (startIndex + i) % MAX_LOG_ENTRIES;
    if (serialLogs[idx].length() > 0) {
      logs += serialLogs[idx] + "\n";
    }
  }
  
  server.send(200, "text/plain", logs);
}

// Handle clear request
void handleClear() {
  // Clear all log entries
  for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
    serialLogs[i] = "";
  }
  logIndex = 0;
  addToSerialLog("Logs cleared");
  
  server.send(200, "text/plain", "Logs cleared");
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