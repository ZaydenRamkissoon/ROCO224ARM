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

void setup() {
  // Start serial communication
  Serial.begin(115200);
  Serial.println("\nESP32 Remote Serial Monitor");
  
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
  
  // Start the server
  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Connect to WiFi SSID: " + String(ssid) + " with password: " + String(password));
  Serial.println("Then visit http://192.168.4.1 in your browser");
  
  // Add initial logs
  addToSerialLog("ESP32 Remote Serial Monitor started");
  addToSerialLog("Access Point IP: " + IP.toString());
}

void loop() {
  // Handle client requests
  server.handleClient();
  
  // Check for serial input periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastSerialCheck > 100) {  // Check every 100ms
    lastSerialCheck = currentMillis;
    
    // Read from serial and add to logs
    while (Serial.available() > 0) {
      String line = Serial.readStringUntil('\n');
      addToSerialLog(line);
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
  html += "<title>ESP32 Remote Serial Monitor</title>";
  html += "<style>";
  html += "body { font-family: monospace; margin: 20px; background-color: #f0f0f0; }";
  html += "h1 { color: #0066cc; }";
  html += "#logContainer { background-color: #1e1e1e; color: #dcdcdc; padding: 10px; border-radius: 5px; height: 70vh; overflow-y: scroll; margin-bottom: 10px; white-space: pre-wrap; }";
  html += ".controls { margin: 10px 0; }";
  html += "button { background-color: #4CAF50; border: none; color: white; padding: 10px 15px; text-align: center; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 5px; }";
  html += "#clearBtn { background-color: #f44336; }";
  html += "</style>";
  html += "<script>";
  html += "let lastUpdate = 0;";
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
  html += "function clearLogs() {";
  html += "  fetch('/clear')";
  html += "    .then(() => {";
  html += "      document.getElementById('logContainer').innerHTML = '';";
  html += "    });";
  html += "}";
  html += "// Initial update and set interval";
  html += "updateLogs();";
  html += "setInterval(updateLogs, 500);";
  html += "</script>";
  html += "</head>";
  html += "<body>";
  html += "<h1>ESP32 Remote Serial Monitor</h1>";
  html += "<div class='controls'>";
  html += "<button onclick='updateLogs()'>Refresh</button>";
  html += "<button id='clearBtn' onclick='clearLogs()'>Clear</button>";
  html += "</div>";
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