#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// Access point credentials
const char* ssid = "ESP32-AP";  // Name of your access point
const char* password = "12345678";  // Password (must be at least 8 characters)

// Web server on port 80
WebServer server(80);

// Define the hardware serial ports for monitoring
// Serial0 is used for programming/debugging (USB) - this is COM16 at 115200 baud
// We'll use Serial2 for connecting to COM8 at 9600 baud
HardwareSerial SerialCOM8(2); // Using UART2

// Pins for the second serial connection (can be adjusted based on your wiring)
#define RX2_PIN 16  // GPIO16 as RX for second serial
#define TX2_PIN 17  // GPIO17 as TX for second serial

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
  // Start main serial communication (USB/COM16)
  Serial.begin(115200);  // Set to 115200 to match COM16 baud rate
  Serial.println("\nESP32 Dual Serial Monitor");
  
  // Start second serial port for COM8 (RX2, TX2 pins)
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
  server.on("/clear", handleClear);
  server.on("/coordinates", handleCoordinates);
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Connect to WiFi SSID: " + String(ssid) + " with password: " + String(password));
  Serial.println("Then visit http://192.168.4.1 in your browser");
  
  // Add initial logs
  addToSerialLog("ESP32 Dual Serial Monitor started");
  addToSerialLog("Main Serial (COM16): 115200 baud");
  addToSerialLog("COM8 Serial: 9600 baud on pins RX:" + String(RX2_PIN) + ", TX:" + String(TX2_PIN));
  addToSerialLog("Access Point IP: " + IP.toString());
}

void loop() {
  // Handle client requests
  server.handleClient();
  
  // Check for serial input periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialCheck > 100) {  // Check every 100ms
    lastSerialCheck = currentMillis;
    
    // Read from main serial (USB/COM16)
    checkSerial(Serial, "COM16");
    
    // Read from COM8 serial
    checkSerial(SerialCOM8, "COM8");
  }
}

// Check a serial port for data
void checkSerial(Stream &serial, String portName) {
  while (serial.available() > 0) {
    char inChar = serial.read();
    
    // Process for face coordinates
    if (processSerialByte(inChar, portName)) {
      // If a full coordinate was processed, don't add to raw log
      continue;
    }
    
    // Add raw serial data to logs if it's a printable character or newline
    if ((inChar >= 32 && inChar <= 126) || inChar == '\n' || inChar == '\r') {
      static String currentLine = "";
      
      if (inChar == '\n' || inChar == '\r') {
        if (currentLine.length() > 0) {
          addToSerialLog(portName + ": " + currentLine);
          currentLine = "";
        }
      } else {
        currentLine += inChar;
      }
    }
  }
}

// Process incoming serial byte for coordinate detection
// Returns true if a complete coordinate was processed
bool processSerialByte(char inChar, String portName) {
  static char coordBuffer[16];
  static int coordIndex = 0;
  static bool inCoordinate = false;
  
  // Start of coordinate packet
  if (inChar == '(') {
    coordIndex = 0;
    coordBuffer[coordIndex++] = inChar;
    inCoordinate = true;
    return true;
  }
  // Add character to buffer
  else if (inCoordinate && coordIndex > 0 && coordIndex < 15) {
    coordBuffer[coordIndex++] = inChar;
    
    // Check if we have complete coordinates
    if (inChar == ')') {
      coordBuffer[coordIndex] = '\0'; // Null terminate
      parseCoordinates(coordBuffer, coordIndex, portName);
      inCoordinate = false;
      return true;
    }
    return true;
  }
  
  return false;
}

// Parse coordinates from buffer in format "(xxx,yyy,zzz)"
void parseCoordinates(char* data, int length, String portName) {
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
      String coordData = String(data);
      String coordMsg = "Face detected from " + portName + ": " + coordData + " (X:" + String(x) + ", Y:" + String(y) + ", Z:" + String(z) + ")";
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
  html += "<meta http-equiv='refresh' content='300'>";  // Refresh page every 5 minutes to prevent stale
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
  html += ".source-note { font-size: 12px; color: #666; font-style: italic; margin-top: 5px; }";
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
  html += "<div class='source-note'>Monitoring COM16 (115200 baud) and COM8 (9600 baud)</div>";
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
      logs += serialLogs[idx] + "<br>";
    }
  }
  
  server.send(200, "text/html", logs);
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