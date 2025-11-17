// Thêm các define cho biểu đồ
#define GRAPH_X 40
#define GRAPH_Y 120
#define GRAPH_WIDTH 270
#define GRAPH_HEIGHT 110
#define MAX_POINTS 300 // Số điểm tối đa có thể lưu trữ

// Thêm các define mới
#define HOLD_TIME 2000 // Thời gian nhấn giữ 3 giây


// Thêm các thư viện WiFi và OTA
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <EEPROM.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <FastPID.h>
#include <ROBOTECH_GP26pt7b.h> 
#include <ROBOTECH_GP48pt7b.h>
#include <ROBOTECH_GP32pt7b.h>
#include <ROBOTECH_GP9pt7b.h>
#include <ROBOTECH_GP12pt7b.h>
#include <ROBOTECH_GP16pt7b.h>
#include <ROBOTECH_GP24pt7b.h>
#include <ROBOTECH_GP20pt7b.h>
// Thêm địa chỉ EEPROM cho Quick Temp
#define EEPROM_QUICKTEMP_START 32 // Bắt đầu từ address 32
#define SENSOR 36
#define VOL 34

#define ENCODER_SW_PIN 15  // Chân nút nhấn encoder
#define SLEEP_BUTTON_PIN 12  // Thêm chân button sleep mới
// Thêm các define cho WiFi và OTA
#define FIRMWARE_URL "https://github.com/bear1712/Tram-Han-Firmware/raw/refs/heads/main/firmware/newwww.ino.bin"
#define AP_SSID "Tram Han"
#define AP_PASSWORD "12345678"
  
WebServer server(80);

// Biến WiFi và trạng thái
String wifiSSID = "";
String wifiPassword = "";
bool wifiConnected = false;
bool apMode = false;
bool updateInProgress = false;
// Thêm biến version hiện tại
String currentFirmwareVersion = "v1.0.1"; // Version hiện tại của bạn
unsigned long apStartTime = 0;
const unsigned long AP_TIMEOUT = 300000; // 5 phút

int wifiRetryCount = 0;
const int MAX_WIFI_RETRY = 3;

// Thêm define cho EEPROM
#define EEPROM_WIFI_START 100
#define EEPROM_WIFI_SIZE 128




// Biến để theo dõi trạng thái nút nhấn
unsigned long buttonPressTime = 0;
// Thêm các giá trị mặc định
const int DEFAULT_MIN_TEMP = 50;
const int DEFAULT_MAX_TEMP = 500;
const int DEFAULT_SLEEP_TEMP = 150;
const int DEFAULT_SETPOINT = 250;
const int DEFAULT_THEME = 0;
const int DEFAULT_QUICK_TEMP[3] = {350, 400, 450};

// Thêm ở phần khai báo biến toàn cục
enum MenuState { MAIN_MENU, SYSTEM_MENU, TOOL_MENU, QUICKTEMP_MENU, THEME_MENU };
MenuState currentMenuState = MAIN_MENU;
// Thêm vào phần khai báo biến toàn cục
bool usePlotterInterface = false; // false = giao diện thường, true = giao diện plotter

// Thêm mảng lưu trữ dữ liệu biểu đồ
int tempHistory[MAX_POINTS];
int setpointHistory[MAX_POINTS];
int pwmHistory[MAX_POINTS];
int historyIndex = 0;
bool graphInitialized = false;
static uint8_t pwmOutput = 0;              // Giá trị PWM output

// Các tham số bộ lọc Kalman
float Q = 0.2;  // Process noise covariance
float R = 1.0;   // Measurement noise covariance
float P = 1.0;   // Estimation error covariance
float K = 0.0;   // Kalman gain
float X = 0.0;   // Estimated value

// Thêm các biến toàn cục
static int filteredRawADC = 0;       // Giá trị đã lọc
static int lastDisplayedRawADC = 0;  // Giá trị cuối cùng hiển thị
const float FILTER_FACTOR = 0.05f;    // Hệ số lọc (0.1-0.3)
const int DISPLAY_THRESHOLD = 2;     // Ngưỡng thay đổi để hiển thị lại

///PID
static int lastPWM = -1;          // Lưu giá trị PWM lần trước
static int lastTempDiff = -100;   // Lưu hiệu nhiệt độ lần trước
const int pwmThreshold = 5;       // Ngưỡng thay đổi PWM
const int tempDiffThreshold = 2;  // Ngưỡng thay đổi nhiệt độ

// Cấu hình màn hình
#define TFT_CS 5
#define TFT_DC 17
#define TFT_RST 16

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const float VOLTAGE_REF = 3.25f;    
const float ADC_RESOLUTION = 4095.0f;
const float R1 = 99000.0f;                // Điện trở trên
const float R2 = 9960.0f;                 // Điện trở dưới
const float VOLTAGE_DIVIDER_RATIO = R2 / (R1 + R2);
float voltage = 0.0f;               // Biến lưu điện áp đo được
const float CALIBRATION_OFFSET = 1.0f; // Hiệu chỉnh sai số cố định

// Cấu hình PID
float Kp = 9, Ki = 0.055, Kd = 1.5, Hz = 20;
FastPID myPID(Kp, Ki, Kd, Hz, 8, false);

// Cấu hình phần cứng
const int pwmPin = 19;
const int encoderPinA = 4;
const int encoderPinB = 2;

// Biến đọc chỉnh setpoint
volatile int encoderPos = 0;
int rawADC = 0;      // Giá trị ADC đọc được
int setpoint = 250;  // Nhiệt độ đặt (khởi đầu 250)
const int min_temp = 50;
const int max_temp = 500;
bool heatingEnabled = true;
int minTemp = 50;
int maxTemp = 500;
// Các chế độ
enum SystemMode { MODE_SETUP, MODE_GOOT, MODE_TX1, MODE_TX2, MODE_T12, MODE_C245, MODE_C210, MODE_SLEEP, MODE_MENU};
SystemMode currentMode = MODE_GOOT;

// Cài đặt sleep
int sleepTemperature = 150;  // Nhiệt độ khi sleep

// Biến kiểm tra mũi
bool solderingIronConnected = true;
const int ADC_DISCONNECT_THRESHOLD = 700;
unsigned long lastDisconnectTime = 0;
bool shouldBlink = false;

// Menu settings - GIỮ NGUYÊN MENU CỦA BẠN
const char *mainMenu[] = {
  "System Settings",
  "Tool Settings",
  "Theme Style",
  "System Reset",
  "System Info",
  "Exit"
};
int menuIndex = 0;
int menuLength = sizeof(mainMenu) / sizeof(mainMenu[0]);
bool inMenu = false;
int quickTemp[3] = {350, 400, 450};  // Nhiệt độ mặc định
int themeStyle = 0; // 0 = Basic, 1 = Chart
// Thêm biến để lưu trạng thái quick temp
int currentQuickIndex = 0;
bool quickTempMode = false;

// Hàm setup() với thứ tự đúng
void setup() {
  Serial.begin(115200);

  // Khởi tạo EEPROM
  EEPROM.begin(512);
  
  // Đọc giá trị từ EEPROM - ĐÚNG THỨ TỰ
  setpoint = readSetpointFromEEPROM();
  sleepTemperature = loadSleepSettings();
  loadMinMaxSettings();
  loadQuickTempSettings();
  loadThemeSettings(); // Đọc theme settings
  loadWiFiSettings(); // ĐỌC WIFI TRƯỚC
  // Khởi tạo màn hình
  tft.init(240, 320, SPI_MODE3);
  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  
  // Khởi tạo PWM
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 0);

  // Cấu hình encoder
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  // Cấu hình nút nhấn
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  pinMode(SLEEP_BUTTON_PIN, INPUT_PULLUP);
  
  // Cấu hình PID
  myPID.setOutputRange(0, 255);
  
  if (usePlotterInterface) {
    drawplotter();
  } else {
    drawInterface();
  }
  // Vẽ giao diện ban đầu
  //drawInterface();
}

void updateEncoder() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  
  // Tăng debounce lên 10ms để ổn định hơn
  if (interruptTime - lastInterruptTime < 10) return;
  lastInterruptTime = interruptTime;

  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  
  old_AB <<= 2;
  old_AB |= (digitalRead(encoderPinA) << 1) | digitalRead(encoderPinB);
  
  int8_t direction = enc_states[(old_AB & 0x0F)];
  
  if (direction != 0) {
    static unsigned long lastTurnTime = 0;
    unsigned long currentTime = millis();
    
    // Giảm độ nhạy của encoder
    encoderPos += direction;
    lastTurnTime = currentTime;
  }
}

// Hàm lưu Quick Temp vào EEPROM
void saveQuickTempSettings() {
  for (int i = 0; i < 3; i++) {
    EEPROM.put(EEPROM_QUICKTEMP_START + (i * sizeof(int)), quickTemp[i]);
  }
  if (EEPROM.commit()) {
    Serial.println("Quick Temp settings saved: " + 
                  String(quickTemp[0]) + ", " + 
                  String(quickTemp[1]) + ", " + 
                  String(quickTemp[2]));
  } else {
    Serial.println("Error saving Quick Temp settings!");
  }
}
// Hàm đọc Quick Temp từ EEPROM
void loadQuickTempSettings() {
  for (int i = 0; i < 3; i++) {
    int tempValue;
    EEPROM.get(EEPROM_QUICKTEMP_START + (i * sizeof(int)), tempValue);
    
    // Kiểm tra giá trị hợp lệ
    if (tempValue >= 150 && tempValue <= 500) {
      quickTemp[i] = tempValue;
    } else {
      quickTemp[i] = DEFAULT_QUICK_TEMP[i]; // Giá trị mặc định
    }
  }
  Serial.println("Quick Temp settings loaded: " + 
                String(quickTemp[0]) + ", " + 
                String(quickTemp[1]) + ", " + 
                String(quickTemp[2]));
}



// Hàm lưu min/max settings
void saveMinMaxSettings() {
  EEPROM.put(24, minTemp);
  EEPROM.put(28, maxTemp);
  if (EEPROM.commit()) {
    Serial.println("Min/Max settings saved: Min=" + String(minTemp) + ", Max=" + String(maxTemp));
  } else {
    Serial.println("Error saving Min/Max settings!");
  }
}
// Hàm đọc min/max settings
void loadMinMaxSettings() {
  int savedMin, savedMax;
  EEPROM.get(24, savedMin);
  EEPROM.get(28, savedMax);
  
  // Kiểm tra giá trị hợp lệ
  if (savedMin >= 0 && savedMin <= 300) {
    minTemp = savedMin;
  } else {
    minTemp = DEFAULT_MIN_TEMP; // Sử dụng giá trị mặc định
  }
  
  if (savedMax >= 200 && savedMax <= 500 && savedMax > minTemp) {
    maxTemp = savedMax;
  } else {
    maxTemp = DEFAULT_MAX_TEMP; // Sử dụng giá trị mặc định
  }
  
  Serial.println("Min/Max settings loaded: Min=" + String(minTemp) + ", Max=" + String(maxTemp));
}


int loadSleepSettings() {
  int temp;
  EEPROM.get(4, temp);
  if (temp < 100 || temp > 250) {
    temp = DEFAULT_SLEEP_TEMP; // Sử dụng giá trị mặc định
    saveSleepSettings(); // Lưu giá trị mặc định nếu không hợp lệ
  }
  return temp;
}

// Cập nhật hàm readSetpointFromEEPROM() để sử dụng giá trị mặc định
int readSetpointFromEEPROM() {
  int tempValue;
  EEPROM.get(0, tempValue);
  if (tempValue < min_temp || tempValue > max_temp) {
    tempValue = DEFAULT_SETPOINT; // Sử dụng giá trị mặc định
  }
  return tempValue;
}

void saveThemeSettings() {
  EEPROM.put(20, themeStyle);
  EEPROM.commit();
  Serial.println("Theme saved: " + String(themeStyle));
}

// Thêm hàm để đọc cài đặt theme (trong setup())
void loadThemeSettings() {
  int savedTheme;
  EEPROM.get(20, savedTheme);
  
  // Kiểm tra giá trị hợp lệ
  if (savedTheme >= 0 && savedTheme <= 1) {
    themeStyle = savedTheme;
  } else {
    themeStyle = DEFAULT_THEME; // Sử dụng giá trị mặc định
    saveThemeSettings(); // Lưu giá trị mặc định
  }
  
  usePlotterInterface = (themeStyle == 1);
  Serial.println("Theme loaded: " + String(themeStyle));
}

void saveSleepSettings() {
  sleepTemperature = constrain(sleepTemperature, 100, 250);
  EEPROM.put(4, sleepTemperature);
  EEPROM.commit();
}

void saveSetpointToEEPROM() {
  setpoint = constrain(setpoint, minTemp, maxTemp); // Sử dụng minTemp và maxTemp
  EEPROM.put(0, setpoint);
  EEPROM.commit();
}

void startConfigAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    
    apMode = true;
    apStartTime = millis();
    
    // Cấu hình các route web
    server.on("/", handleRoot);
    server.on("/config", handleConfig);
    server.on("/save", handleSave);
    server.on("/scan", handleScan);
    server.on("/status", handleStatus);
    
    server.begin();
    Serial.println("HTTP server started");
    
    // Hiển thị thông báo trên màn hình - SỬA VỊ TRÍ VÀ FONT
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(10, 40);  // VỊ TRÍ GIỐNG MENU
    tft.print("CONFIGURATION MODE");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
    
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 90);  // BẮT ĐẦU TỪ VỊ TRÍ 90
    tft.print("SSID: ");
    tft.print(AP_SSID);
    tft.setCursor(10, 120);
    tft.print("Password: ");
    tft.print(AP_PASSWORD);
    tft.setCursor(10, 150);
    tft.print("IP: ");
    tft.print(myIP);
    
    tft.setCursor(10, 190);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Connect to WiFi and");
    tft.setCursor(10, 210);
    tft.print("visit the IP address");
}

void handleRoot() {
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>Tram Han WiFi Config</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 500px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .form-group { margin-bottom: 15px; }
        label { display: block; margin-bottom: 5px; font-weight: bold; }
        input[type="text"], input[type="password"] { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; }
        button { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .scan-btn { background: #28a745; margin-bottom: 10px; }
        .scan-btn:hover { background: #1e7e34; }
        .network-list { margin-top: 10px; }
        .network-item { padding: 8px; border: 1px solid #ddd; margin-bottom: 5px; border-radius: 4px; cursor: pointer; }
        .network-item:hover { background: #f8f9fa; }
        .status { padding: 10px; border-radius: 4px; margin-bottom: 15px; }
        .success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        /* THÊM STYLE CHO PASSWORD TOGGLE */
        .password-container { position: relative; }
        .toggle-password { 
            position: absolute; 
            right: 10px; 
            top: 50%; 
            transform: translateY(-50%); 
            background: none; 
            border: none; 
            cursor: pointer; 
            color: #666;
            font-size: 12px;
            padding: 4px 8px;
        }
        .toggle-password:hover { 
            background: #f0f0f0; 
            border-radius: 3px;
            color: #333; 
        }
    </style>
</head>
<body>
    <div class="container">
        <h2>Tram Han WiFi Configuration</h2>
        
        <div id="status"></div>
        
        <button class="scan-btn" onclick="scanNetworks()">Scan WiFi Networks</button>
        <div id="networks" class="network-list"></div>
        
        <form onsubmit="saveConfig(event)">
            <div class="form-group">
                <label for="ssid">WiFi SSID:</label>
                <input type="text" id="ssid" name="ssid" required>
            </div>
            
            <div class="form-group">
                <label for="password">WiFi Password:</label>
                <div class="password-container">
                    <input type="password" id="password" name="password">
                    <button type="button" class="toggle-password" onclick="togglePassword()">Show</button>
                </div>
            </div>
            
            <button type="submit">Save & Connect</button>
        </form>
        
        <div style="margin-top: 20px; font-size: 12px; color: #666;">
            <p><strong>Current Status:</strong> <span id="currentStatus">Not connected</span></p>
            <p><strong>Saved SSID:</strong> <span id="savedSSID">None</span></p>
        </div>
    </div>

    <script>
        // Load saved settings khi trang load
        window.onload = function() {
            loadStatus();
        }

        function loadStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('currentStatus').textContent = data.wifiConnected ? 'Connected' : 'Disconnected';
                    document.getElementById('savedSSID').textContent = data.savedSSID || 'None';
                    if (data.savedSSID) {
                        document.getElementById('ssid').value = data.savedSSID;
                    }
                });
        }

        function scanNetworks() {
            fetch('/scan')
                .then(response => response.json())
                .then(networks => {
                    const networksDiv = document.getElementById('networks');
                    networksDiv.innerHTML = '<h3>Available Networks:</h3>';
                    
                    networks.forEach(network => {
                        const div = document.createElement('div');
                        div.className = 'network-item';
                        div.innerHTML = `${network.ssid}`;
                        div.onclick = () => {
                            document.getElementById('ssid').value = network.ssid;
                        };
                        networksDiv.appendChild(div);
                    });
                });
        }

        // THÊM HÀM TOGGLE PASSWORD
        function togglePassword() {
            const passwordInput = document.getElementById('password');
            const toggleButton = document.querySelector('.toggle-password');
            
            if (passwordInput.type === 'password') {
                passwordInput.type = 'text';
                toggleButton.textContent = 'Hide';
            } else {
                passwordInput.type = 'password';
                toggleButton.textContent = 'Show';
            }
        }

        function saveConfig(event) {
            event.preventDefault();
            
            const ssid = document.getElementById('ssid').value;
            const password = document.getElementById('password').value;
            
            const formData = new FormData();
            formData.append('ssid', ssid);
            formData.append('password', password);
            
            fetch('/save', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(data => {
                showStatus(data.message, data.success ? 'success' : 'error');
                if (data.success) {
                    setTimeout(() => {
                        loadStatus();
                    }, 2000);
                }
            });
        }

        function showStatus(message, type) {
            const statusDiv = document.getElementById('status');
            statusDiv.innerHTML = `<div class="status ${type}">${message}</div>`;
            setTimeout(() => {
                statusDiv.innerHTML = '';
            }, 5000);
        }
    </script>
</body>
</html>
)=====";
    
    server.send(200, "text/html", html);
}

void handleScan() {
    String json = "[";
    int n = WiFi.scanNetworks();
    
    for (int i = 0; i < n; ++i) {
        if (i) json += ",";
        json += "{";
        json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
        json += "\"encrypted\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        json += "}";
    }
    json += "]";
    
    server.send(200, "application/json", json);
}

void handleConfig() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }
    
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    
    if (ssid.length() == 0) {
        server.send(400, "application/json", "{\"success\": false, \"message\": \"SSID cannot be empty\"}");
        return;
    }
    
    // Lưu cài đặt WiFi - SỬA CHỖ NÀY
    wifiSSID = ssid;
    wifiPassword = password;
    saveWiFiSettings();  // THÊM DÒNG NÀY ĐỂ LUÔN LUÔN LƯU
    
    // Thử kết nối WiFi
    String message;
    bool success = false;
    
    if (connectToWiFi()) {
        message = "Successfully connected to " + ssid;
        success = true;
        
        // Thoát chế độ AP sau khi kết nối thành công
        apMode = false;
        WiFi.mode(WIFI_STA);
    } else {
        message = "Failed to connect to " + ssid;
        // NHƯNG VẪN LƯU SETTINGS NGAY CẢ KHI KẾT NỐI THẤT BẠI
        success = true; // Vẫn trả về success=true vì đã lưu settings
    }
    
    String response = "{\"success\": " + String(success ? "true" : "false") + 
                     ", \"message\": \"" + message + "\"}";
    server.send(200, "application/json", response);
}

void handleSave() {
    handleConfig(); // Gọi cùng hàm với handleConfig
}

void handleStatus() {
    String json = "{";
    json += "\"wifiConnected\": " + String(wifiConnected ? "true" : "false") + ",";
    json += "\"savedSSID\": \"" + wifiSSID + "\",";
    json += "\"apMode\": " + String(apMode ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
}









void connectWithCustomWiFi() {
    if (wifiSSID.length() == 0) {
        showMessage("Error", "SSID cannot be empty!");
        return;
    }
    
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(10, 40);  // VỊ TRÍ GIỐNG MENU
    tft.print("CONNECTING TO WIFI");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
    
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 90);  // BẮT ĐẦU TỪ VỊ TRÍ 90
    tft.print("Connecting to:");
    tft.setCursor(10, 120);
    tft.print(wifiSSID);
    
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    
    unsigned long startTime = millis();
    int dots = 0;
    
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
        delay(500);
        tft.fillRect(280, 120, 40, 20, ST77XX_BLACK);  // ĐIỀU CHỈNH VỊ TRÍ DOTS
        tft.setCursor(280, 120);
        for (int i = 0; i < dots; i++) {
            tft.print(".");
        }
        dots = (dots + 1) % 4;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        tft.setCursor(10, 160);
        tft.print("Connected!");
        tft.setCursor(10, 190);
        tft.print("IP: ");
        tft.print(WiFi.localIP());
        delay(3000);
    } else {
        tft.setCursor(10, 160);
        tft.print("Failed to connect!");
        delay(3000);
    }
}





void saveWiFiSettings() {
    Serial.println("Saving WiFi settings to EEPROM...");
    Serial.println("SSID: " + wifiSSID);
    Serial.println("Password: " + wifiPassword);
    
    // XÓA VÙNG EEPROM CŨ TRƯỚC KHI LƯU
    for (int i = EEPROM_WIFI_START; i < EEPROM_WIFI_START + 128; i++) {
        EEPROM.write(i, 0);
    }
    
    // LƯU SSID
    for (int i = 0; i < wifiSSID.length(); i++) {
        EEPROM.write(EEPROM_WIFI_START + i, wifiSSID[i]);
    }
    EEPROM.write(EEPROM_WIFI_START + wifiSSID.length(), '\0');
    
    // LƯU PASSWORD
    for (int i = 0; i < wifiPassword.length(); i++) {
        EEPROM.write(EEPROM_WIFI_START + 64 + i, wifiPassword[i]);
    }
    EEPROM.write(EEPROM_WIFI_START + 64 + wifiPassword.length(), '\0');
    
    if (EEPROM.commit()) {
        Serial.println("WiFi settings saved successfully!");
        showMessage("Success", "WiFi settings saved!\nSSID: " + wifiSSID);
    } else {
        Serial.println("Error saving WiFi settings!");
        showMessage("Error", "Failed to save WiFi settings!");
    }
}

void loadWiFiSettings() {
    Serial.println("Loading WiFi settings from EEPROM...");
    
    // ĐỌC SSID
    String savedSSID = "";
    for (int i = EEPROM_WIFI_START; i < EEPROM_WIFI_START + 64; i++) {
        char c = EEPROM.read(i);
        if (c == 0 || c == 255) break;
        savedSSID += c;
    }
    
    // ĐỌC PASSWORD
    String savedPassword = "";
    for (int i = EEPROM_WIFI_START + 64; i < EEPROM_WIFI_START + 128; i++) {
        char c = EEPROM.read(i);
        if (c == 0 || c == 255) break;
        savedPassword += c;
    }
    
    // Kiểm tra xem có dữ liệu hợp lệ không
    if (savedSSID.length() > 0 && savedSSID.length() < 64) {
        wifiSSID = savedSSID;
        wifiPassword = savedPassword;
        Serial.println("Loaded WiFi settings:");
        Serial.println("SSID: " + wifiSSID);
        
        // SỬA: Sử dụng String để nối chuỗi
        Serial.println("Password: " + String(savedPassword.length() > 0 ? "***" : "<empty>"));
    } else {
        Serial.println("No valid WiFi settings found in EEPROM");
        wifiSSID = "";
        wifiPassword = "";
    }
}

void showMessage(const char* title, String message) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(10, 40);
    tft.print(title);
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
    
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 90);
    
    // Xử lý message có nhiều dòng
    int lineHeight = 25;
    int currentY = 90;
    
    // Tách message thành các dòng nếu có \n
    int startPos = 0;
    int newlinePos;
    do {
        newlinePos = message.indexOf('\n', startPos);
        String line;
        if (newlinePos == -1) {
            line = message.substring(startPos);
        } else {
            line = message.substring(startPos, newlinePos);
        }
        
        tft.setCursor(10, currentY);
        tft.print(line);
        currentY += lineHeight;
        startPos = newlinePos + 1;
    } while (newlinePos != -1 && currentY < 220);
    
    //tft.setCursor(10, 220);
    //tft.setTextColor(ST77XX_CYAN);
    //tft.print("Press button to continue...");
    
    while (digitalRead(ENCODER_SW_PIN) == HIGH) {
        delay(100);
    }
    while (digitalRead(ENCODER_SW_PIN) == LOW) {
        delay(100);
    }
}


void handleEncoderButton() {
  static unsigned long lastPressTime = 0;
  static bool buttonActive = false;
  
  if (currentMode == MODE_SLEEP || inMenu) return;
  
  bool isPressed = (digitalRead(ENCODER_SW_PIN) == LOW);
  
  if (isPressed && !buttonActive) {
    // Nút vừa được nhấn
    buttonPressTime = millis();
    buttonActive = true;
  } 
  else if (!isPressed && buttonActive) {
    // Nút vừa được thả
    buttonActive = false;
    unsigned long pressDuration = millis() - buttonPressTime;
    
    if (pressDuration < HOLD_TIME) {
      // NHẤN NGẮN - Chuyển đổi giữa các quick temp
      if (quickTempMode) {
        currentQuickIndex = (currentQuickIndex + 1) % 3; // 0, 1, 2
        setpoint = quickTemp[currentQuickIndex];
      } else {
        // Kích hoạt quick temp mode lần đầu
        quickTempMode = true;
        currentQuickIndex = 0;
        setpoint = quickTemp[0];
      }
      
      // Hiển thị thông báo quick temp
      tft.fillScreen(ST77XX_BLACK);
      tft.setTextColor(ST77XX_YELLOW);
      tft.setFont(&ROBOTECH_GP24pt7b);
      tft.setCursor(70, 100);
      tft.print("QUICK ");
      tft.print(currentQuickIndex + 1);
      tft.setCursor(70, 150);
      tft.print(setpoint);
      tft.print("C");
      delay(800);
      
      // Vẽ lại giao diện
      tft.fillScreen(ST77XX_BLACK);
      if (usePlotterInterface) {
        drawplotter();
      } else {
        drawInterface();
      }
      
      saveSetpointToEEPROM();
    }
    else if (pressDuration >= HOLD_TIME) {
      // NHẤN GIỮ - vào menu
      quickTempMode = false; // Thoát quick temp mode khi vào menu
      enterMenu();
    }
  }
}

void enterMenu() {
  inMenu = true;
  currentMode = MODE_MENU;
  menuIndex = 0;
  encoderPos = 0;
  
  // XÓA MÀN HÌNH TRIỆT ĐỂ
  tft.fillScreen(ST77XX_BLACK);
  delay(5); // Đợi ngắn để đảm bảo xóa hoàn toàn
  
  // VẼ TOÀN BỘ MENU
  tft.setFont(&ROBOTECH_GP16pt7b);
  tft.setTextColor(ST77XX_WHITE);

  for (int i = 0; i < menuLength; i++) {
    int yPos = 40 + i * 35;
    tft.setCursor(15, yPos);
    tft.print(mainMenu[i]);
  }

  // VẼ KHUNG CHO MỤC ĐẦU TIÊN NGAY LẬP TỨC
  int yPos = 40 + menuIndex * 35;
  tft.drawRoundRect(5, yPos - 21, 230, 28, 5, ST77XX_WHITE);
}


void exitMenu() {
  inMenu = false;
  currentMode = MODE_GOOT;
  tft.fillRect(0, 0, 320, 240, ST77XX_BLACK); // Xóa sạch menu
  delay(10);
  
  // Vẽ lại giao diện theo theme đã chọn
  if (usePlotterInterface) {
    drawplotter();
  } else {
    drawInterface();
  }
}

// GIỮ NGUYÊN CÁC HÀM MENU CỦA BẠN
void drawMainMenu(int index) {
  // LUÔN VẼ TOÀN BỘ MENU MỖI LẦN GỌI
  tft.fillRect(0, 0, 320, 240, ST77XX_BLACK);
  delay(10); // Đợi một chút để đảm bảo xóa hoàn toàn
  tft.setFont(&ROBOTECH_GP16pt7b);
  tft.setTextColor(ST77XX_WHITE);

  // Vẽ tất cả các mục menu
  for (int i = 0; i < menuLength; i++) {
    int yPos = 40 + i * 35;
    tft.setCursor(15, yPos);
    tft.print(mainMenu[i]);
  }

  // Vẽ khung cho mục được chọn
  if (index >= 0 && index < menuLength) {
    int yPos = 40 + index * 35;
    tft.drawRoundRect(5, yPos - 21, 230, 28, 5, ST77XX_WHITE);
  }
}

// Trong hàm showSystemSettingsMenu(), sửa để sử dụng biến minTemp và maxTemp
void showSystemSettingsMenu() {
  String tipList[] = {"GOOT", "TX1", "TX2", "T12", "C245", "C210", "C470"};
  int tipIndex = 0;
  bool soundOn = true;

  const char* items[] = {
    "Select Tip",
    "Sound",
    "Minimum Temp",
    "Maximum Temp"
  };
  int itemCount = sizeof(items) / sizeof(items[0]);
  int menuIndex = 0;
  bool inEditMode = false;

  encoderPos = 0;

  // Vị trí hiển thị từng dòng (điều chỉnh để dành chỗ cho tiêu đề)
  int yPos[itemCount];
  for (int i = 0; i < itemCount; i++) {
    yPos[i] = 140 + i * 30; // Điều chỉnh vị trí xuống dưới
  }

  // Vẽ khung nền + nội dung ban đầu
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  
  // Vẽ tiêu đề "SYSTEM SETTINGS" ở vị trí (0, 10)
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 40);
  tft.print("SYSTEM SETTINGS");
  
  // Vẽ đường kẻ ngang dưới tiêu đề
  tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);
  for (int i = 0; i < itemCount; i++) {
    tft.setCursor(10, yPos[i]);
    tft.print(items[i]);
  }

  // Vẽ giá trị ban đầu
  auto drawValue = [&](int i) {
    tft.fillRect(240, yPos[i] - 16, 100, 18, ST77XX_BLACK);  // xóa vùng giá trị cũ
    tft.setCursor(240, yPos[i]);
    if (i == 0) tft.print(tipList[tipIndex]);
    else if (i == 1) tft.print(soundOn ? "ON" : "OFF");
    else if (i == 2) { tft.print(minTemp); tft.print(" C"); }
    else if (i == 3) { tft.print(maxTemp); tft.print(" C"); }
  };
  for (int i = 0; i < itemCount; i++) drawValue(i);

  unsigned long lastActivity = millis();  // Lưu thời điểm cuối có thao tác
  // Vẽ khung chọn mục ban đầu
  int lastIndex = -1;
  while (true) {
    if (!inEditMode) {
      menuIndex = abs(encoderPos) % itemCount;
    }

    // Chỉ vẽ lại nếu menuIndex thay đổi
    if (menuIndex != lastIndex) {
    // Xóa khung cũ (bằng cách vẽ đè màu nền)
    if (lastIndex >= 0) {
      tft.drawRoundRect(5, yPos[lastIndex] - 21, 190, 28, 5, ST77XX_BLACK);
    }

    // Vẽ khung trắng cho mục đang chọn
    tft.drawRoundRect(5, yPos[menuIndex] - 21, 190, 28, 5, ST77XX_WHITE);
    lastIndex = menuIndex;
    lastActivity = millis();  // Cập nhật thời gian hoạt động
    }

    // Nếu đang chỉnh, cập nhật giá trị tương ứng
    if (inEditMode) {
      switch (menuIndex) {
        case 0: {
          int newIndex = abs(encoderPos) % 7;
          if (newIndex != tipIndex) {
            tipIndex = newIndex;
            drawValue(0);
          }
          break;
        }
        case 1: {
          bool newSound = encoderPos % 2 == 0;
          if (newSound != soundOn) {
            soundOn = newSound;
            drawValue(1);
          }
          break;
        }
        case 2: { // Minimum Temp
          int newMin = constrain(encoderPos, 0, maxTemp - 50); // Đảm bảo min < max
          if (newMin != minTemp) {
            minTemp = newMin;
            drawValue(2);
          }
          break;
        }
        case 3: { // Maximum Temp
          int newMax = constrain(encoderPos, minTemp + 50, 500); // Đảm bảo max > min
          if (newMax != maxTemp) {
            maxTemp = newMax;
            drawValue(3);
          }
          break;
        }
      }
    }

    // Nhấn nút để chuyển chế độ
    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(50);
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        if (inEditMode) {
          inEditMode = false;
          encoderPos = menuIndex;
          
          // KHI THOÁT CHẾ ĐỈNH SỬA MIN/MAX, LƯU LẠI GIÁ TRỊ
          if (menuIndex == 2 || menuIndex == 3) {
            saveMinMaxSettings(); // Lưu khi thoát chỉnh sửa
          }
        } else {
          inEditMode = true;
          switch (menuIndex) {
            case 0: encoderPos = tipIndex; break;
            case 1: encoderPos = soundOn ? 0 : 1; break;
            case 2: encoderPos = minTemp; break;
            case 3: encoderPos = maxTemp; break;
          }
        }
        while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
        lastActivity = millis();  // Cập nhật thời gian hoạt động
      }
    }

     // Thoát nếu không hoạt động trong 2 giây
    if (millis() - lastActivity > 5000) {
      // LƯU MIN/MAX SETTINGS KHI THOÁT MENU
      saveMinMaxSettings();
      break;
    }

    delay(30);
  }

  drawMainMenu(menuIndex);  // trở về menu chính
}


void showQuickTempMenu() {
  currentMenuState = QUICKTEMP_MENU;
  
  const int itemCount = 3;
  const char* labels[] = { "Quick 1", "Quick 2", "Quick 3" };
  unsigned long lastActivity = millis();
  int menuIndex = 0;
  bool inEditMode = false;
  encoderPos = 0;

  int yPos[itemCount];
  for (int i = 0; i < itemCount; i++) {
    yPos[i] = 140 + i * 30;
  }

  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  
  // Vẽ tiêu đề
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 40);
  tft.print("QUICK TEMP SETTINGS");
  tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);
  for (int i = 0; i < itemCount; i++) {
    tft.setCursor(10, yPos[i]);
    tft.print(labels[i]);
  }

  auto drawValue = [&](int i) {
    tft.fillRect(240, yPos[i] - 16, 100, 18, ST77XX_BLACK);
    tft.setCursor(240, yPos[i]);
    tft.print(quickTemp[i]);
    tft.print(" C");
  };

  for (int i = 0; i < itemCount; i++) drawValue(i);

  int lastIndex = -1;

  while (currentMenuState == QUICKTEMP_MENU) { // Sử dụng biến trạng thái
    if (!inEditMode) {
      menuIndex = abs(encoderPos) % itemCount;
    }

    if (menuIndex != lastIndex) {
      if (lastIndex >= 0)
        tft.drawRoundRect(5, yPos[lastIndex] - 21, 100, 28, 5, ST77XX_BLACK);
      tft.drawRoundRect(5, yPos[menuIndex] - 21, 100, 28, 5, ST77XX_WHITE);
      lastIndex = menuIndex;
      lastActivity = millis();
    }

    if (inEditMode) {
      int val = constrain(encoderPos, 150, 500);
      if (val != quickTemp[menuIndex]) {
        quickTemp[menuIndex] = val;
        drawValue(menuIndex);
        lastActivity = millis();
        saveQuickTempSettings();
      }
    }

    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(50);
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        if (inEditMode) {
          inEditMode = false;
          encoderPos = menuIndex;
        } else {
          inEditMode = true;
          encoderPos = quickTemp[menuIndex];
        }
        while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
        lastActivity = millis();
      }
    }

    // THOÁT VỀ MENU CHÍNH SAU 5 GIÂY
    if (millis() - lastActivity > 5000) {
      saveQuickTempSettings();
      currentMenuState = MAIN_MENU; // Thoát về menu chính
      break;
    }

    delay(50);
  }
  
  // KHÔNG GỌI showToolSettingsMenu() Ở ĐÂY NỮA
  // Chỉ cần thoát khỏi hàm, điều khiển sẽ trở về nơi gọi
}



void showToolSettingsMenu() {
  currentMenuState = TOOL_MENU;
  
  int sleepTemp = 200;
  int sleepDelay = 30;
  int standDelay = 60;

  const char* items[] = {
    "Quick Temp",
    "Sleep Temp",
    "Sleep Delay",
    "Stand Delay"
  };
  int itemCount = sizeof(items) / sizeof(items[0]);

  int menuIndex = 0;
  bool inEditMode = false;
  encoderPos = 0;

  int yPos[itemCount];
  for (int i = 0; i < itemCount; i++) {
    yPos[i] = 140 + i * 30;
  }

  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  
  // Vẽ tiêu đề
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 40);
  tft.print("TOOL SETTINGS");
  tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);
  for (int i = 0; i < itemCount; i++) {
    tft.setCursor(10, yPos[i]);
    tft.print(items[i]);
  }

  auto drawValue = [&](int i) {
    tft.fillRect(220, yPos[i] - 16, 100, 18, ST77XX_BLACK);
    tft.setCursor(220, yPos[i]);
    switch (i) {
      case 0: tft.print(">>"); break;
      case 1: tft.print(sleepTemp); tft.print(" C"); break;
      case 2: tft.print(sleepDelay); tft.print(" min"); break;
      case 3: tft.print(standDelay); tft.print(" min"); break;
    }
  };

  for (int i = 0; i < itemCount; i++) drawValue(i);

  int lastIndex = -1;
  unsigned long lastActivity = millis();

  while (currentMenuState == TOOL_MENU) { // Sử dụng biến trạng thái
    if (!inEditMode) {
      int newIndex = abs(encoderPos) % itemCount;
      if (newIndex != menuIndex) {
        menuIndex = newIndex;
        lastActivity = millis();
      }
    }

    if (menuIndex != lastIndex) {
      if (lastIndex >= 0)
        tft.drawRoundRect(5, yPos[lastIndex] - 21, 190, 28, 5, ST77XX_BLACK);
      tft.drawRoundRect(5, yPos[menuIndex] - 21, 190, 28, 5, ST77XX_WHITE);
      lastIndex = menuIndex;
    }

    if (inEditMode && menuIndex != 0) {
      switch (menuIndex) {
        case 1: {
          int newVal = constrain(encoderPos, 100, 300);
          if (newVal != sleepTemp) {
            sleepTemp = newVal;
            drawValue(1);
            lastActivity = millis();
          }
          break;
        }
        case 2: {
          int newVal = constrain(encoderPos, 0, 30);
          if (newVal != sleepDelay) {
            sleepDelay = newVal;
            drawValue(2);
            lastActivity = millis();
          }
          break;
        }
        case 3: {
          int newVal = constrain(encoderPos, 0, 60);
          if (newVal != standDelay) {
            standDelay = newVal;
            drawValue(3);
            lastActivity = millis();
          }
          break;
        }
      }
    }

    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(50);
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        if (menuIndex == 0) {
          // Vào Quick Temp menu
          showQuickTempMenu();
          
          // Sau khi thoát từ Quick Temp, kiểm tra xem cần về đâu
          if (currentMenuState == TOOL_MENU) {
            // Vẽ lại Tool Settings
            tft.fillScreen(ST77XX_BLACK);
            tft.setTextColor(ST77XX_YELLOW);
            tft.setCursor(10, 40);
            tft.print("TOOL SETTINGS");
            tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
            tft.setTextColor(ST77XX_WHITE);
            for (int i = 0; i < itemCount; i++) {
              tft.setCursor(10, yPos[i]);
              tft.print(items[i]);
              drawValue(i);
            }
            lastIndex = -1;
            encoderPos = menuIndex;
            lastActivity = millis();
          }
          continue;
        }

        if (inEditMode) {
          inEditMode = false;
          encoderPos = menuIndex;
        } else {
          inEditMode = true;
          switch (menuIndex) {
            case 1: encoderPos = sleepTemp; break;
            case 2: encoderPos = sleepDelay; break;
            case 3: encoderPos = standDelay; break;
          }
        }
        while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
        lastActivity = millis();
      }
    }

    // THOÁT VỀ MENU CHÍNH SAU 5 GIÂY
    if (millis() - lastActivity > 5000) {
      currentMenuState = MAIN_MENU;
      break;
    }

    delay(50);
  }

  // Chỉ vẽ main menu nếu đang ở trạng thái MAIN_MENU
  if (currentMenuState == MAIN_MENU) {
    drawMainMenu(1);
  }
}

void showThemeStyleMenu() {
  const char* items[] = {"Basic", "Chart"};
  const int itemCount = sizeof(items) / sizeof(items[0]);
  int yPos[itemCount];
  for (int i = 0; i < itemCount; i++) {
    yPos[i] = 140 + i * 30; // Điều chỉnh vị trí xuống dưới
  }

  encoderPos = themeStyle;
  int currentSelection = themeStyle;
  int lastSelection = -1;
  unsigned long lastActivity = millis();
  bool inEditMode = false;

  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  
  // Vẽ tiêu đề "THEME SETTINGS" ở vị trí (0, 10)
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 40);
  tft.print("THEME SETTINGS");
  
  // Vẽ đường kẻ ngang dưới tiêu đề
  tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

  tft.setTextColor(ST77XX_WHITE);

  // Hàm vẽ item (sửa lại để phù hợp với vị trí mới)
  auto drawItem = [&](int i, bool isHighlighted) {
    tft.fillRect(0, yPos[i] - 22, 320, 30, ST77XX_BLACK);
    
    if (isHighlighted) {
      tft.drawRoundRect(5, yPos[i] - 21, 310, 28, 5, ST77XX_WHITE);
    }
    
    tft.setCursor(20, yPos[i]);
    tft.setTextColor(ST77XX_WHITE);
    tft.print(items[i]);
    
    if (themeStyle == i) {
      tft.setCursor(200, yPos[i]);
      tft.setTextColor(ST77XX_GREEN);
      tft.print("Selected");
    }
  };

  // Vẽ tất cả items lần đầu
  for (int i = 0; i < itemCount; i++) {
    drawItem(i, i == currentSelection);
  }

  while (true) {
    // Cập nhật selection từ encoder
    if (!inEditMode) {
      currentSelection = abs(encoderPos) % itemCount;
      if (currentSelection < 0) currentSelection = 0;
      if (currentSelection >= itemCount) currentSelection = itemCount - 1;
    }

    // Chỉ vẽ lại nếu selection thay đổi
    if (currentSelection != lastSelection) {
      if (lastSelection >= 0) {
        drawItem(lastSelection, false);
      }
      drawItem(currentSelection, true);
      lastSelection = currentSelection;
      lastActivity = millis();
    }

    // Xử lý nút nhấn
    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(50); // Debounce
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        // Chọn theme này
        themeStyle = currentSelection;
        usePlotterInterface = (themeStyle == 1); // 0 = Basic, 1 = Chart
        
        // Cập nhật hiển thị
        for (int i = 0; i < itemCount; i++) {
          drawItem(i, i == currentSelection);
        }
        
        // Lưu cài đặt theme
        saveThemeSettings();
        
        // Hiển thị thông báo
        tft.fillRect(0, 180, 320, 40, ST77XX_BLACK);
        tft.setCursor(80, 200);
        tft.setTextColor(ST77XX_GREEN);
        tft.print("Theme Saved!");
        
        delay(1000);
        break;
      }
    }

    // Timeout sau 5 giây không hoạt động
    if (millis() - lastActivity > 5000) {
      break;
    }

    delay(50);
  }
  // KHI THOÁT MENU CON, XÓA SẠCH VÀ QUAY LẠI MENU CHÍNH
  tft.fillRect(0, 0, 320, 240, ST77XX_BLACK);
  delay(10);
  drawMainMenu(menuIndex);

}

// Trong hàm confirmFactoryReset()
void confirmFactoryReset() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  
  // Vẽ tiêu đề "FACTORY RESET" ở vị trí (0, 10)
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 40);
  tft.print("FACTORY RESET");
  
  // Vẽ đường kẻ ngang dưới tiêu đề
  tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

  tft.setCursor(20, 100);
  tft.setTextColor(ST77XX_RED);
  tft.print("Factory Reset?");
  tft.setCursor(20, 140);
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Press to confirm");
  tft.setCursor(20, 180);
  tft.print("Hold to cancel");

  unsigned long startTime = millis();
  bool confirmed = false;
  
  while (millis() - startTime < 5000) { // 3 giây để xác nhận
    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(50);
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        confirmed = true;
        break;
      }
    }
    delay(100);
  }

  if (!confirmed) {
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(20, 120);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Reset Cancelled");
    delay(1000);
    drawMainMenu(menuIndex);
    return;
  }

  // THỰC HIỆN RESET VỀ MẶC ĐỊNH
  // 1. Reset Quick Temp
  for (int i = 0; i < 3; i++) {
    quickTemp[i] = DEFAULT_QUICK_TEMP[i];
  }
  saveQuickTempSettings(); // Lưu giá trị mặc định


  // 2. Reset Min/Max Temp
  minTemp = DEFAULT_MIN_TEMP;
  maxTemp = DEFAULT_MAX_TEMP;
  saveMinMaxSettings();

  // 3. Reset Sleep Temperature
  sleepTemperature = DEFAULT_SLEEP_TEMP;
  saveSleepSettings();

  // 4. Reset Setpoint
  setpoint = DEFAULT_SETPOINT;
  saveSetpointToEEPROM();

  // 5. Reset Theme
  themeStyle = DEFAULT_THEME;
  usePlotterInterface = (themeStyle == 1);
  saveThemeSettings();

  // 6. Reset các cài đặt khác nếu có
  // ...

  // Hiển thị thông báo thành công
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(20, 100);
  tft.setTextColor(ST77XX_GREEN);
  tft.print("Reset Complete!");
  tft.setCursor(20, 140);
  tft.print("All settings reset");
  tft.setCursor(20, 180);
  tft.print("to default values");

  Serial.println("Factory reset completed. All settings set to default.");
  Serial.println("MinTemp: " + String(minTemp) + ", MaxTemp: " + String(maxTemp));
  Serial.println("SleepTemp: " + String(sleepTemperature));
  Serial.println("Setpoint: " + String(setpoint));
  Serial.println("Theme: " + String(themeStyle));

  delay(2000);
  drawMainMenu(menuIndex);
}
void checkAndShowUpdate() {
    String updateStatus = checkFirmwareVersion();
    
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 40);
    tft.print("FIRMWARE UPDATE CHECK");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

    tft.setTextColor(ST77XX_WHITE);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(10, 90);
    tft.print("Current Version: " + currentFirmwareVersion);
    tft.setCursor(10, 120);
    tft.print("Available: ");
    
    // HIỂN THỊ TRẠNG THÁI ĐÚNG
    if (updateStatus == "WiFi Failed") {
        tft.setTextColor(ST77XX_RED);
        tft.print("WiFi Failed!");
    } else if (updateStatus == "Check Failed") {
        tft.setTextColor(ST77XX_RED);
        tft.print("Check Failed!");
    } else if (updateStatus == "Latest") {
        tft.setTextColor(ST77XX_GREEN);
        tft.print(currentFirmwareVersion);
        
        tft.setCursor(10, 160);
        tft.setTextColor(ST77XX_YELLOW);
        tft.print("You have the latest version");
    } else {
        // CÓ BẢN MỚI
        tft.setTextColor(ST77XX_GREEN);
        tft.print(updateStatus);
        
        tft.setCursor(10, 160);
        tft.setTextColor(ST77XX_RED);
        tft.print("New version available!");
        tft.setCursor(10, 190);
        tft.print("Go to Update Firmware");
    }

    waitForButtonPress();
}

void confirmFirmwareUpdate() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_RED);
    tft.setFont(&ROBOTECH_GP16pt7b);
    
    // SỬA: Căn trái giống các menu khác (10 thay vì 50)
    tft.setCursor(10, 40);  // <-- SỬA TỪ 80 XUỐNG 40 VÀ 10 THAY VÌ 50
    tft.print("FIRMWARE UPDATE");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);  // <-- SỬA TỪ 90 XUỐNG 50

    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(10, 90);   // <-- ĐIỀU CHỈNH VỊ TRÍ XUỐNG
    tft.print("This will update your");
    tft.setCursor(10, 120);  // <-- ĐIỀU CHỈNH VỊ TRÍ XUỐNG
    tft.print("firmware to latest version");
    
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(30, 190);  // <-- ĐIỀU CHỈNH VỊ TRÍ XUỐNG
    tft.print("Press to CONFIRM");
    tft.setCursor(30, 220);  // <-- ĐIỀU CHỈNH VỊ TRÍ XUỐNG
    tft.print("Hold to CANCEL");

    unsigned long startTime = millis();
    bool confirmed = false;
    
    while (millis() - startTime < 5000) {
        if (digitalRead(ENCODER_SW_PIN) == LOW) {
            delay(50);
            if (digitalRead(ENCODER_SW_PIN) == LOW) {
                confirmed = true;
                break;
            }
        }
        delay(100);
    }

    if (confirmed) {
        performOTAUpdate();
    } else {
        tft.fillScreen(ST77XX_BLACK);
        tft.setCursor(50, 120);
        tft.setTextColor(ST77XX_YELLOW);
        tft.print("Update Cancelled");
        delay(1000);
    }
}

void showWiFiStatus() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 40);
    tft.print("WIFI STATUS");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

    tft.setTextColor(ST77XX_WHITE);
    tft.setFont(&ROBOTECH_GP16pt7b);
    
    // Hiển thị trạng thái
    tft.setCursor(10, 90);
    tft.print("Status: ");
    if (wifiConnected) {
        tft.setTextColor(ST77XX_GREEN);
        tft.print("CONNECTED");
    } else {
        tft.setTextColor(ST77XX_RED);
        tft.print("DISCONNECTED");
    }
    
    tft.setTextColor(ST77XX_WHITE);
    
    if (wifiConnected) {
        // Hiển thị SSID
        tft.setCursor(10, 130);
        tft.print("SSID: ");
        tft.print(WiFi.SSID());
        
        // Hiển thị IP
        tft.setCursor(10, 170);
        tft.print("IP: ");
        tft.print(WiFi.localIP());
    } else {
        // Hiển thị SSID đã lưu (nếu có)
        tft.setCursor(10, 130);
        tft.print("Saved SSID: ");
        if (wifiSSID.length() > 0) {
            tft.print(wifiSSID);
        } else {
            tft.print("None");
        }
        
        tft.setCursor(10, 170);
        tft.print("Use WiFi Config");
        tft.setCursor(10, 200);
        tft.print("to connect");
    }

    waitForButtonPress();
}

void waitForButtonPress() {
    //tft.setCursor(10, 250);
    //tft.setTextColor(ST77XX_CYAN);
    //tft.setFont(&ROBOTECH_GP12pt7b);
    //tft.print("Press button to continue...");
    
    // Chờ nút nhấn
    while (digitalRead(ENCODER_SW_PIN) == HIGH) {
        delay(100);
    }
    // Debounce
    while (digitalRead(ENCODER_SW_PIN) == LOW) {
        delay(100);
    }
    
    // ✅ THÊM: XÓA THÔNG BÁO VÀ ĐƯỜNG KẺ
    //tft.fillRect(5, 230, 300, 10, ST77XX_BLACK); // Xóa dòng thông báo
    //tft.fillRect(10, 50, 300, 2, ST77XX_BLACK);   // Xóa đường kẻ ngang
}

// Hàm hỗ trợ cho WiFi Configuration (cần được thêm vào)
void showWiFiConfigMenu() {
    const char* wifiItems[] = {
        "Start Config AP",
        "Connect WiFi", 
        "WiFi Status",
        "Back"
    };
    int wifiItemCount = sizeof(wifiItems) / sizeof(wifiItems[0]);
    int wifiMenuIndex = 0;
    encoderPos = 0;

    tft.fillScreen(ST77XX_BLACK);
    tft.setFont(&ROBOTECH_GP16pt7b);
    
    // Tiêu đề - VẼ 1 LẦN DUY NHẤT
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 40);
    tft.print("WIFI CONFIGURATION");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

    // VẼ TOÀN BỘ MENU ITEMS 1 LẦN DUY NHẤT
    tft.setTextColor(ST77XX_WHITE);
    for (int i = 0; i < wifiItemCount; i++) {
        int yPos = 90 + i * 30;
        tft.setCursor(15, yPos);
        tft.print(wifiItems[i]);
    }

    unsigned long lastActivity = millis();
    int lastIndex = -1;
    
    while (true) {
        // Cập nhật menu index từ encoder
        int newIndex = encoderPos % wifiItemCount;
        if (newIndex < 0) newIndex += wifiItemCount;
        
        if (newIndex != wifiMenuIndex) {
            // XÓA KHUNG CŨ (chỉ xóa khung, không xóa toàn bộ menu)
            if (lastIndex >= 0) {
                int oldYPos = 90 + lastIndex * 30;
                tft.drawRoundRect(5, oldYPos - 21, 230, 28, 5, ST77XX_BLACK);
            }

            // Cập nhật menuIndex
            wifiMenuIndex = newIndex;

            // VẼ KHUNG MỚI
            int newYPos = 90 + wifiMenuIndex * 30;
            tft.drawRoundRect(5, newYPos - 21, 230, 28, 5, ST77XX_WHITE);

            lastIndex = wifiMenuIndex;
            lastActivity = millis();
        }

        // Xử lý nút nhấn
        if (digitalRead(ENCODER_SW_PIN) == LOW) {
            delay(50); // Debounce
            if (digitalRead(ENCODER_SW_PIN) == LOW) {
                
                // LƯU LẠI TRẠNG THÁI MENU HIỆN TẠI
                int savedMenuIndex = wifiMenuIndex;
                
                switch (wifiMenuIndex) {
                    case 0: // Start Config AP
                            enterAPMode();
                            break;//showMessage("AP Started", "Connect to TramHan_Config\nPassword: 12345678");
                        
                    case 1: // Connect WiFi
                        if (wifiSSID.length() > 0) {
                            connectWithCustomWiFi();
                        } else {
                            showMessage("Error", "No saved WiFi settings!\nStart Config AP first.");
                        }
                        break;
                    case 2: // WiFi Status
                        showWiFiStatus();
                        break;
                    case 3: // Back
                        return;
                }
                
                // KHÔI PHỤC HOÀN TOÀN GIAO DIỆN SAU KHI QUAY LẠI
                tft.fillScreen(ST77XX_BLACK);
                tft.setFont(&ROBOTECH_GP16pt7b);
                
                // Vẽ lại tiêu đề
                tft.setTextColor(ST77XX_YELLOW);
                tft.setCursor(10, 40);
                tft.print("WIFI CONFIGURATION");
                tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
                
                // Vẽ lại toàn bộ menu items
                tft.setTextColor(ST77XX_WHITE);
                for (int i = 0; i < wifiItemCount; i++) {
                    int yPos = 90 + i * 30;
                    tft.setCursor(15, yPos);
                    tft.print(wifiItems[i]);
                }
                
                // Vẽ lại khung cho mục được chọn
                int yPos = 90 + savedMenuIndex * 30;
                tft.drawRoundRect(5, yPos - 21, 230, 28, 5, ST77XX_WHITE);
                
                // Đặt lại trạng thái
                lastIndex = savedMenuIndex;
                wifiMenuIndex = savedMenuIndex;
                encoderPos = savedMenuIndex;
                lastActivity = millis();
            }
            while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
        }

        // Xử lý Web Server nếu đang ở chế độ AP
        if (apMode) {
            server.handleClient();
            
            // Tự động thoát AP sau 5 phút
            if (millis() - apStartTime > AP_TIMEOUT) {
                apMode = false;
                WiFi.mode(WIFI_STA);
                showMessage("AP Timeout", "Configuration mode ended");
                
                // KHÔI PHỤC GIAO DIỆN SAU KHI THOÁT AP
                tft.fillScreen(ST77XX_BLACK);
                tft.setFont(&ROBOTECH_GP16pt7b);
                
                // Vẽ lại tiêu đề
                tft.setTextColor(ST77XX_YELLOW);
                tft.setCursor(10, 40);
                tft.print("WIFI CONFIGURATION");
                tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
                
                // Vẽ lại toàn bộ menu items
                tft.setTextColor(ST77XX_WHITE);
                for (int i = 0; i < wifiItemCount; i++) {
                    int yPos = 90 + i * 30;
                    tft.setCursor(15, yPos);
                    tft.print(wifiItems[i]);
                }
                
                // Vẽ lại khung cho mục được chọn
                int yPos = 90 + wifiMenuIndex * 30;
                tft.drawRoundRect(5, yPos - 21, 230, 28, 5, ST77XX_WHITE);
                
                lastActivity = millis();
            }
        }

        // Timeout sau 30 giây không hoạt động
        if (millis() - lastActivity > 30000) {
            return;
        }

        delay(50);
    }
}


void enterAPMode() {
    startConfigAP();  // ← HÀM NÀY ĐÃ VẼ GIAO DIỆN RỒI

    // Vòng lặp chính của AP mode
    unsigned long startTime = millis();
    while (millis() - startTime < 300000) { // 5 phút timeout
        server.handleClient();
        
        if (digitalRead(ENCODER_SW_PIN) == LOW) {
            delay(250);
            break;
        }
        
        // Kiểm tra nếu đã kết nối WiFi thành công
        if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            
            // XÓA DÒNG "Press button to exit" TRƯỚC KHI HIỂN THỊ THÔNG BÁO MỚI
            tft.fillRect(10, 175, 300, 60, ST77XX_BLACK); // Xóa vùng thông báo cũ
            
            tft.setCursor(10, 190);
            tft.setTextColor(ST77XX_GREEN);
            tft.print("WiFi Connected!");
            tft.setCursor(10, 220);
            tft.print("Returning to main...");
            
            delay(2000);
            break;
        }
        
        delay(100);
    }
    
    // Dọn dẹp
    apMode = false;
    WiFi.mode(WIFI_STA);
}

void showDeviceInfo() {
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 40);
    tft.print("DEVICE INFORMATION");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

    tft.setTextColor(ST77XX_WHITE);
    tft.setFont(&ROBOTECH_GP12pt7b);
    
    // Hiển thị thông tin thiết bị - SỬA: DÙNG BIẾN TOÀN CỤC
    tft.setCursor(10, 90);
    tft.print("Firmware: Tram Han " + currentFirmwareVersion);
    tft.setCursor(10, 120);
    tft.print("Author: Nguyen Hoang Nam");
    
    tft.setCursor(10, 150);
    tft.print("ESP32 Chip ID: ");
    tft.print((uint32_t)ESP.getEfuseMac(), HEX);
    
    tft.setCursor(10, 180);
    tft.print("Free Memory: ");
    tft.print(ESP.getFreeHeap());
    tft.print(" bytes");

    waitForButtonPress();
}
// ==============================================
// CÁC HÀM WiFi VÀ OTA CÒN THIẾU
// ==============================================

bool connectToWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }
    
    if (wifiSSID.length() == 0) {
        Serial.println("No WiFi SSID configured");
        return false;
    }
    
    Serial.println("Connecting to: " + wifiSSID);
    
    WiFi.disconnect();
    delay(1000);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    
    unsigned long startTime = millis();
    int attempts = 0;
    
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        if (millis() - startTime > 15000) { // 15 giây timeout
            break;
        }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.println("\nCONNECTED! IP: " + WiFi.localIP().toString());
        return true;
    } else {
        Serial.println("\nCONNECTION FAILED!");
        wifiConnected = false;
        return false;
    }
}

String checkFirmwareVersion() {
    if (!connectToWiFi()) {
        return "WiFi Failed";
    }
    
    HTTPClient http;
    
    String versionURL = "https://raw.githubusercontent.com/bear1712/Tram-Han-Firmware/refs/heads/main/firmware/firmware.version";
    
    Serial.println("Checking version from: " + versionURL);
    
    http.begin(versionURL);
    http.setTimeout(10000);
    http.setUserAgent("TramHan-Firmware-Checker");
    
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 80);
    tft.print("Checking update...");
    
    int httpCode = http.GET();
    Serial.println("HTTP Response: " + String(httpCode));
    
    if (httpCode == HTTP_CODE_OK) {
        String newVersion = http.getString();
        newVersion.trim();
        
        Serial.println("Available version: " + newVersion);
        Serial.println("Current version: " + currentFirmwareVersion);
        
        http.end();
        
        // SO SÁNH CHÍNH XÁC - LOẠI BỎ KHOẢNG TRẮNG
        newVersion.trim();
        String currentTrimmed = currentFirmwareVersion;
        currentTrimmed.trim();
        
        Serial.println("Comparing: '" + newVersion + "' vs '" + currentTrimmed + "'");
        
        if (newVersion == currentTrimmed) {
            return "Latest";
        } else {
            return newVersion; // Có bản mới
        }
    } else {
        Serial.println("HTTP Error: " + String(httpCode));
    }
    
    http.end();
    return "Check Failed";
}

void performOTAUpdate() {
    if (!connectToWiFi()) {
        showMessage("Error", "WiFi Connection Failed");
        return;
    }
    
    HTTPClient http;
    String firmwareURL = FIRMWARE_URL;
    
    Serial.println("=== OTA UPDATE START ===");
    Serial.println("URL: " + firmwareURL);
    
    http.begin(firmwareURL);
    http.setTimeout(60000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    
    // VẼ GIAO DIỆN BAN ĐẦU - BỐ CỤC RÕ RÀNG
    tft.fillScreen(ST77XX_BLACK);
    
    // ===== PHẦN 1: TIÊU ĐỀ =====
    tft.setTextColor(ST77XX_YELLOW);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(10, 30);
    tft.print("UPDATE FIRMWARE");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
    
    // ===== PHẦN 2: TRẠNG THÁI KẾT NỐI =====
    tft.setTextColor(ST77XX_WHITE);
    tft.setFont(&ROBOTECH_GP12pt7b);
    tft.setCursor(10, 70);
    tft.print("Connecting...");
    
    int httpCode = http.GET();
    Serial.println("HTTP Response: " + String(httpCode));
    
    // XÓA DÒNG TRẠNG THÁI CŨ
    tft.fillRect(10, 55, 300, 20, ST77XX_BLACK);
    tft.setCursor(10, 70);
    
    if (httpCode != HTTP_CODE_OK) {
        tft.setTextColor(ST77XX_RED);
        tft.print("Connection Failed: " + String(httpCode));
        http.end();
        delay(2000);
        showMessage("Error", "Download Failed\nCode: " + String(httpCode));
        return;
    }
    
    // KẾT NỐI THÀNH CÔNG
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Connected! Downloading...");
    
    int contentLength = http.getSize();
    Serial.println("File Size: " + String(contentLength) + " bytes");
    
    if (contentLength <= 0) {
        http.end();
        showMessage("Error", "Invalid File: " + String(contentLength) + " bytes");
        return;
    }
    
    // KIỂM TRA BỘ NHỚ
    if (contentLength > ESP.getFreeSketchSpace()) {
        http.end();
        showMessage("Error", "Not enough space!");
        return;
    }
    
// ===== PHẦN 3: THÔNG TIN FILE =====
tft.fillRect(10, 100, 300, 20, ST77XX_BLACK); // XÓA VÙNG THÔNG TIN FILE
tft.setCursor(10, 100);
tft.setTextColor(ST77XX_WHITE);
tft.print("File size: " + String(contentLength) + " bytes");

// ===== PHẦN 4: PROGRESS DISPLAY =====
tft.fillRect(10, 130, 300, 20, ST77XX_BLACK); // XÓA VÙNG PROGRESS TEXT
tft.setCursor(10, 130);
tft.print("Progress: ");

// PROGRESS BAR BACKGROUND - VẼ MỚI HOÀN TOÀN
tft.fillRect(10, 150, 300, 25, ST77XX_BLACK); // XÓA TOÀN BỘ PROGRESS BAR
tft.drawRect(10, 150, 300, 25, ST77XX_WHITE);

// PERCENTAGE DISPLAY - XÓA SẠCH TRƯỚC KHI VẼ
tft.fillRect(260, 130, 50, 20, ST77XX_BLACK);
tft.setCursor(260, 130);
tft.print("0%");

if (Update.begin(contentLength)) {
    WiFiClient* stream = http.getStreamPtr();
    size_t written = 0;
    uint8_t buffer[512];
    unsigned long lastProgressUpdate = 0;
    
    while (http.connected() && written < contentLength) {
        int size = stream->available();
        if (size > 0) {
            int c = stream->readBytes(buffer, min((size_t)size, sizeof(buffer)));
            if (c > 0) {
                written += Update.write(buffer, c);
                
                // CHỈ CẬP NHẬT PROGRESS MỖI 200ms
                if (millis() - lastProgressUpdate > 200) {
                    int progress = (written * 100) / contentLength;
                    
                    // ===== CẬP NHẬT PHẦN TRĂM =====
                    tft.fillRect(255, 115, 50, 20, ST77XX_BLACK); // XÓA VÙNG PHẦN TRĂM CŨ
                    tft.setCursor(260, 130);
                    tft.setTextColor(ST77XX_CYAN);
                    tft.print(progress);
                    tft.print("%");
                    
                    // ===== CẬP NHẬT PROGRESS BAR =====
                    // XÓA TOÀN BỘ PROGRESS BAR TRƯỚC KHI VẼ LẠI
                    tft.fillRect(11, 151, 298, 23, ST77XX_BLACK); // Xóa bên trong progress bar
                    tft.drawRect(10, 150, 300, 25, ST77XX_WHITE); // Vẽ lại viền
                    
                    int barWidth = (298 * progress) / 100; // Giảm 2px để không chạm viền
                    if (barWidth > 0) {
                        tft.fillRect(11, 151, barWidth, 23, ST77XX_GREEN);
                    }
                    
                    // ===== HIỂN THỊ BYTES ĐÃ TẢI =====
                    tft.fillRect(10, 195, 300, 20, ST77XX_BLACK); // XÓA TOÀN BỘ VÙNG BYTES
                    tft.setCursor(10, 210);
                    tft.setTextColor(ST77XX_WHITE);
                    tft.print(String(written) + "/" + String(contentLength) + " bytes");
                    
                    Serial.println("Progress: " + String(progress) + "% - " + 
                                 String(written) + "/" + String(contentLength) + " bytes");
                    
                    lastProgressUpdate = millis();
                }
            }
        }
        delay(1);
    } 
        
        Serial.println("Download completed: " + String(written) + " bytes");
        
        // ===== XÓA TOÀN BỘ VÙNG HIỂN THỊ CŨ =====
        tft.fillRect(0, 75, 320, 145, ST77XX_BLACK);
        
        if (Update.end()) {
            Serial.println("Update SUCCESS!");
             // CẬP NHẬT VERSION MỚI TỪ FILE version ĐÃ DOWNLOAD

            // ===== PHẦN 5: THÔNG BÁO THÀNH CÔNG =====
            tft.setTextColor(ST77XX_GREEN);
            tft.setFont(&ROBOTECH_GP16pt7b);
            tft.setCursor(50, 100);
            tft.print("UPDATE SUCCESS!");
            
            tft.setFont(&ROBOTECH_GP12pt7b);
            tft.setCursor(30, 130);
            tft.print("Rebooting in 3 seconds...");
            
            // ===== HIỂN THỊ ĐẾM NGƯỢC =====
            for (int i = 3; i > 0; i--) {
                tft.fillRect(160, 145, 40, 30, ST77XX_BLACK);
                tft.setCursor(160, 160);
                tft.setTextColor(ST77XX_YELLOW);
                tft.setFont(&ROBOTECH_GP16pt7b);
                tft.print(i);
                delay(1000);
            }
            
            ESP.restart();
            
        } else {
            String errorMsg = "Flash Failed: ";
            errorMsg += String(Update.getError());
            errorMsg += " - ";
            errorMsg += Update.errorString();
            Serial.println(errorMsg);
            
            // ===== PHẦN 6: HIỂN THỊ LỖI =====
            tft.fillRect(0, 100, 320, 100, ST77XX_BLACK);
            tft.setTextColor(ST77XX_RED);
            tft.setCursor(10, 120);
            tft.print("Flash Failed!");
            tft.setCursor(10, 150);
            tft.print("Error: " + String(Update.getError()));
            delay(3000);
            showMessage("Flash Failed", errorMsg);
        }
    } else {
        showMessage("Error", "Update.begin() failed!");
    }
    
    http.end();
}


void showSystemInfo() {
    const char* infoItems[] = {
        "Device Info",
        "WiFi Config", 
        "Check Update", 
        "Update Firmware",
        "Back"
    };
    int infoItemCount = sizeof(infoItems) / sizeof(infoItems[0]);
    int infoMenuIndex = 0;
    encoderPos = 0;

    tft.fillScreen(ST77XX_BLACK);
    tft.setFont(&ROBOTECH_GP16pt7b);
    
    // Tiêu đề - VẼ 1 LẦN DUY NHẤT
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 40);
    tft.print("SYSTEM INFORMATION");
    tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);

    // VẼ TOÀN BỘ MENU ITEMS 1 LẦN DUY NHẤT
    tft.setTextColor(ST77XX_WHITE);
    for (int i = 0; i < infoItemCount; i++) {
        int yPos = 90 + i * 30;
        tft.setCursor(15, yPos);
        tft.print(infoItems[i]);
    }

    unsigned long lastActivity = millis();
    int lastIndex = -1;
    
    while (true) {
        // Cập nhật menu index từ encoder
        int newIndex = encoderPos % infoItemCount;
        if (newIndex < 0) newIndex += infoItemCount;
        
        if (newIndex != infoMenuIndex) {
            // XÓA KHUNG CŨ (chỉ xóa khung, không xóa toàn bộ menu)
            if (lastIndex >= 0) {
                int oldYPos = 90 + lastIndex * 30;
                tft.drawRoundRect(5, oldYPos - 21, 230, 28, 5, ST77XX_BLACK);
            }

            // Cập nhật menuIndex
            infoMenuIndex = newIndex;

            // VẼ KHUNG MỚI
            int newYPos = 90 + infoMenuIndex * 30;
            tft.drawRoundRect(5, newYPos - 21, 230, 28, 5, ST77XX_WHITE);

            lastIndex = infoMenuIndex;
            lastActivity = millis();
        }

        // Xử lý nút nhấn
        if (digitalRead(ENCODER_SW_PIN) == LOW) {
            delay(50); // Debounce
            if (digitalRead(ENCODER_SW_PIN) == LOW) {
                
                // LƯU LẠI TRẠNG THÁI MENU HIỆN TẠI
                int savedMenuIndex = infoMenuIndex;
                
                switch (infoMenuIndex) {
                    case 0: // Device Info
                        showDeviceInfo();
                        break;
                    case 1: // WiFi Configuration
                        showWiFiConfigMenu();
                        break;
                    case 2: // Check Update
                        checkAndShowUpdate();
                        break;
                    case 3: // Update Firmware
                        confirmFirmwareUpdate();
                        break;
                    case 4: // Back
                        drawMainMenu(menuIndex);
                        return;
                }
                
                // KHÔI PHỤC HOÀN TOÀN GIAO DIỆN SAU KHI QUAY LẠI
                tft.fillScreen(ST77XX_BLACK);
                tft.setFont(&ROBOTECH_GP16pt7b);
                
                // Vẽ lại tiêu đề
                tft.setTextColor(ST77XX_YELLOW);
                tft.setCursor(10, 40);
                tft.print("SYSTEM INFORMATION");
                tft.drawFastHLine(10, 50, 300, ST77XX_WHITE);
                
                // Vẽ lại toàn bộ menu items
                tft.setTextColor(ST77XX_WHITE);
                for (int i = 0; i < infoItemCount; i++) {
                    int yPos = 90 + i * 30;
                    tft.setCursor(15, yPos);
                    tft.print(infoItems[i]);
                }
                
                // Vẽ lại khung cho mục được chọn
                int yPos = 90 + savedMenuIndex * 30;
                tft.drawRoundRect(5, yPos - 21, 230, 28, 5, ST77XX_WHITE);
                
                // Đặt lại trạng thái
                lastIndex = savedMenuIndex;
                infoMenuIndex = savedMenuIndex;
                encoderPos = savedMenuIndex;
                lastActivity = millis();
            }
            while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
        }

        // Timeout sau 30 giây không hoạt động
        if (millis() - lastActivity > 30000) {
            drawMainMenu(menuIndex);
            return;
        }

        delay(50);
    }
}




void handleMenuNavigation() {
  if (!inMenu) return;
  
  static unsigned long lastUpdateTime = 0;
  const unsigned long UPDATE_INTERVAL = 50; // ms
  
  if (millis() - lastUpdateTime < UPDATE_INTERVAL) {
    return;
  }
  
  // Cập nhật menuIndex từ encoder
  int newIndex = encoderPos % menuLength;
  if (newIndex < 0) newIndex += menuLength;
  
  if (newIndex != menuIndex) {
    // XÓA KHUNG CŨ (vẽ đè màu đen)
    int oldYPos = 40 + menuIndex * 35;
    tft.drawRoundRect(5, oldYPos - 21, 230, 28, 5, ST77XX_BLACK);

    // Cập nhật menuIndex
    menuIndex = newIndex;

    // VẼ KHUNG MỚI
    int newYPos = 40 + menuIndex * 35;
    tft.drawRoundRect(5, newYPos - 21, 230, 28, 5, ST77XX_WHITE);

    lastUpdateTime = millis();
  }
  
  // Xử lý nút nhấn với debounce
  static unsigned long lastButtonPress = 0;
  if (digitalRead(ENCODER_SW_PIN) == LOW && millis() - lastButtonPress > 250) {
    lastButtonPress = millis();
    
    while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
    
    switch (menuIndex) {
      case 0: showSystemSettingsMenu(); break;
      case 1: showToolSettingsMenu(); break;
      case 2: showThemeStyleMenu(); break;
      case 3: confirmFactoryReset(); break;
      case 4: showSystemInfo(); break;
      case 5: exitMenu(); break;
    }
    
    lastUpdateTime = millis();
  }
}





// Hàm exitMenu đã có sẵn trong code trước đó


  void handleSleepButton() {
    static int prevSetpoint = setpoint;
    static bool wasPressed = false;
    static bool prevUsePlotter = usePlotterInterface;
    bool isPressed = (digitalRead(SLEEP_BUTTON_PIN) == LOW);

    if (isPressed && !wasPressed) {
        // Khi nhấn nút sleep
        prevSetpoint = setpoint;
        prevUsePlotter = usePlotterInterface;
        currentMode = MODE_SLEEP;
        usePlotterInterface = false; // QUAN TRỌNG: Tắt chế độ plotter khi vào sleep
      
        historyIndex = 0;
        graphInitialized = false;
        lastDisplayedRawADC = -1;
        tft.fillScreen(ST77XX_BLACK);
        drawSleepModeScreen(); // Vẽ giao diện sleep (đã bao gồm vị trí rawADC mới)
    
    } 
    else if (!isPressed && wasPressed) {
        // Khi thả nút sleep
        currentMode = MODE_GOOT;
        setpoint = prevSetpoint;
        usePlotterInterface = prevUsePlotter; // Khôi phục trạng thái hiển thị
        
        tft.fillScreen(ST77XX_BLACK);
        
        if (usePlotterInterface) {
            // Khởi tạo lại đồ thị plotter
            for (int i = 0; i < MAX_POINTS; i++) {
                tempHistory[i] = rawADC;
                setpointHistory[i] = setpoint;
                pwmHistory[i] = pwmOutput;
            }
            drawplotter();
        } else {
            drawInterface();
        }
        
        // BUỘC CẬP NHẬT NGAY LẬP TỨC
        lastDisplayedRawADC = -1;
        lastPWM = -1;
        updateDisplay1();
        updateDisplay();
        updatePID();
        updateGraph();
    }
    
    wasPressed = isPressed;
  }



 
  /////////////////////////////////////////////////////////////////

  void checkIronConnection() {
      // Kiểm tra ngắt kết nối (phản hồi tức thì)
      if (rawADC > ADC_DISCONNECT_THRESHOLD) {
          if (solderingIronConnected) {
              solderingIronConnected = false;
              shouldBlink = true; // Bật chế độ chớp nháy
              lastDisconnectTime = millis();
          }
      } 
      // Kiểm tra kết nối lại (có độ trễ 500ms để ổn định)
      else if (!solderingIronConnected && (millis() - lastDisconnectTime > 500)) {
          solderingIronConnected = true;
          shouldBlink = false; // Tắt chế độ chớp nháy
          tft.fillRect(1, 2, 160, 45, ST77XX_BLACK);
              if (currentMode == MODE_GOOT) {
              GOOT(); // Vẽ lại giao diện Normal
              } 
              else if (currentMode == MODE_SLEEP) {
              sleep(); // Vẽ lại giao diện Sleep
              }
          
      }
  }


  //////////////////////////////////////////////////////////////////////////////


    void drawplotter() {
    initGraph();
    updateGraph(); 
    updateDisplay1();
    updateVoltageDisplay1();
    updateSetpoint1();
    GOOT1(); 
    drawQuickTempButtons(); // Thêm hiển thị nút Quick Temp
  }


  void drawInterface() {
    updateDisplay();
    updatePID();
    updateVoltageDisplay();
    tft.fillRect(80, 60, 165, 83, ST77XX_BLACK);
    updateSetpoint();
    tft.fillRect(2, 2, 160, 45, ST77XX_BLACK);
    GOOT(); 
    drawQuickTempButtons1();
  }

  void GOOT() {
    tft.setTextColor(ST77XX_WHITE);  
    tft.setFont(&ROBOTECH_GP26pt7b);
    tft.setCursor(5, 40);
    tft.print("GOOT");
  }
    void GOOT1() {
    tft.setTextColor(ST77XX_WHITE);  
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(5, 20);
    tft.print("GOOT");
  }
  void displayIronDisconnected() {
      static unsigned long lastBlinkTime = 0;
      static bool blinkState = false;
      const unsigned long BLINK_INTERVAL = 500; // Tăng tốc độ chớp (200ms)

      if (shouldBlink && (millis() - lastBlinkTime >= BLINK_INTERVAL)) {
          lastBlinkTime = millis();
          blinkState = !blinkState;
          
          if (blinkState) {
              tft.fillRect(2, 2, 110, 45, ST77XX_BLACK);
              tft.setTextColor(ST77XX_YELLOW);
              tft.setFont(&ROBOTECH_GP26pt7b);
              tft.setCursor(5, 40);
              tft.print("S - E");
          } else {
              tft.fillRect(2, 2, 110, 45, ST77XX_BLACK);
              
          }
      }
  }


  void drawSleepModeScreen() {
    updatePID();
    updateDisplay();
    updateVoltageDisplay();
    tft.fillRect(80, 60, 165, 83, ST77XX_BLACK);
    setupsleep();
    tft.fillRect(2, 2, 160, 45, ST77XX_BLACK);
    sleep();
    
    
  }

  void sleep() {
    tft.setTextColor(tft.color565(200, 200, 200));  // Light Sky Blue
    tft.setFont(&ROBOTECH_GP26pt7b);
    tft.setCursor(5, 40);
    tft.print("SLEEP");
  }
  void setupsleep() {
    // Cập nhật màn hình
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setFont(&ROBOTECH_GP24pt7b);
    tft.setCursor(5, 120);
    tft.print("SET:");
    tft.setTextColor(tft.color565(200, 200, 200), ST77XX_BLACK);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(234, 108);
    tft.print("o");
    tft.setFont(&ROBOTECH_GP24pt7b);
    tft.setCursor(253, 115);
    tft.print("C");
    tft.setFont(&ROBOTECH_GP48pt7b);
    // Xóa vùng hiển thị cũ (x=80, y=60, w=165, h=83)
    tft.fillRect(80, 60, 153, 83, ST77XX_BLACK);
    
    // Chuyển số thành chuỗi
    String text = String(sleepTemperature);
    
    // Tính toán kích thước văn bản
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    
    // Tính toán vị trí giữa (trong vùng 165 pixel)
    int16_t x = 80 + (153 - w) / 2;
    
    // Vẽ số ở giữa
    tft.setCursor(x, 140); // Giữ nguyên y=140 như code gốc
    tft.print(text);
  }


  void updateSetpoint() {
    if(usePlotterInterface) return;  // Không cập nhật nếu đang ở chế độ plotter
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setFont(&ROBOTECH_GP24pt7b);
    tft.setCursor(5, 120);
    tft.print("SET:");
    tft.setTextColor(tft.color565(0,191,255) , ST77XX_BLACK);
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(234, 108);
    tft.print("o");
    tft.setFont(&ROBOTECH_GP24pt7b);
    tft.setCursor(253, 115);
    tft.print("C");
    
    
    tft.setFont(&ROBOTECH_GP48pt7b);
    
    // Xóa vùng hiển thị cũ (x=80, y=60, w=165, h=83)
    tft.fillRect(80, 95, 152, 46, ST77XX_BLACK);
    
    // Chuyển số thành chuỗi
    String text = String(setpoint);
    
    // Tính toán kích thước văn bản
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    
    // Tính toán vị trí giữa (trong vùng 165 pixel)
    int16_t x = 82 + (152 - w) / 2;
    
    // Vẽ số ở giữa
    tft.setCursor(x, 140); // Giữ nguyên y=140 như code gốc
    tft.print(text);
  }

void updateSetpoint1() {
  if(!usePlotterInterface) return;
  
  // Cập nhật giá trị setpoint
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  tft.setCursor(215, 100);
  tft.print("SET:");
  tft.setTextColor(tft.color565(0,191,255), ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP16pt7b);
  
  // Xóa vùng hiển thị cũ
  tft.fillRect(264, 84, 51, 18, ST77XX_BLACK);
  
  // Chuyển số thành chuỗi
  String text = String(setpoint);
  
  // Tính toán kích thước văn bản
  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  
  // Tính toán vị trí giữa
  int16_t x = 264 + (51 - w) / 2;
  
  // Vẽ số ở giữa
  tft.setCursor(x, 100);
  tft.print(text);
  
  // Cập nhật màu sắc các nút Quick Temp
  drawQuickTempButtons();
}

// Thêm hàm để cập nhật nút Quick Temp khi thay đổi giá trị
  
  void updateDisplay1() {
    if(!usePlotterInterface) return;
    // Chỉ cập nhật nếu giá trị thay đổi đáng kể
    if(abs(filteredRawADC - lastDisplayedRawADC) >= DISPLAY_THRESHOLD) {
        tft.setFont(&ROBOTECH_GP32pt7b);
        tft.setTextColor(tft.color565(255,255,0), ST77XX_BLACK);
        tft.fillRect(212, 38, 105, 35, ST77XX_BLACK);

        String textreal = String(filteredRawADC);
        
        int16_t x1, y1;
        uint16_t w, h;
        tft.getTextBounds(textreal, 0, 0, &x1, &y1, &w, &h);
        
        int16_t x = 215 + (100 - w) / 2;
        tft.setCursor(x, 70);
        tft.print(textreal);
        
        lastDisplayedRawADC = filteredRawADC; // Cập nhật giá trị cuối cùng
    }
  }

 void updateDisplay() {
  // KHÔNG cập nhật nếu đang trong menu hoặc sleep mode
  if(usePlotterInterface || inMenu || currentMode == MODE_SLEEP) return;
  
  tft.setTextColor(tft.color565(255,255,0), ST77XX_BLACK);
  tft.setFont(&ROBOTECH_GP9pt7b);
  tft.setCursor(185, 170);
  tft.print("o");
  tft.setFont(&ROBOTECH_GP16pt7b);
  tft.setCursor(196, 175);
  tft.print("C");
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(10, 175);
  tft.print("TEMP:");
  
  // Chỉ cập nhật nếu giá trị thay đổi đáng kể
  if(abs(filteredRawADC - lastDisplayedRawADC) >= DISPLAY_THRESHOLD) {
      tft.setFont(&ROBOTECH_GP32pt7b);
      tft.setTextColor(tft.color565(255,255,0), ST77XX_BLACK);
      tft.fillRect(80, 160, 104, 31, ST77XX_BLACK);

      String textreal1 = String(filteredRawADC);
      
      int16_t x1, y1;
      uint16_t w, h;
      tft.getTextBounds(textreal1, 0, 0, &x1, &y1, &w, &h);
      
      int16_t x = 82 + (104 - w) / 2;
      tft.setCursor(x, 190);
      tft.print(textreal1);
      
      lastDisplayedRawADC = filteredRawADC;
  }
}

 // Thêm hàm vẽ nút Quick Temp
void drawQuickTempButtons() {
  if (!usePlotterInterface) return;
  
  //tft.setFont(&ROBOTECH_GP9pt7b);
  
  for (int i = 0; i < 3; i++) {
    int x = 5 + (i * 70);
    int y = 72;
    
    
    // Vẽ viền nút (màu khác nếu đang được chọn)
    //if (quickTempMode && currentQuickIndex == i) {
      //tft.drawRoundRect(x, y, 50, 25, 5, ST77XX_YELLOW);
    //} else {
      //tft.drawRoundRect(x, y, 50, 25, 5, ST77XX_WHITE);
    //}
    
    // Vẽ chữ "Q" và số
    tft.setFont(&ROBOTECH_GP9pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(x + 20, y + 12);
    tft.print("CH");
    tft.print(i + 1);
    
    // Vẽ giá trị nhiệt độ bên dưới
     tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setCursor(x + 10, y + 35);
    tft.print(quickTemp[i]);
    //tft.print("C");
  }
}
 // Thêm hàm vẽ nút Quick Temp
void drawQuickTempButtons1() {
  if (usePlotterInterface) return;
  
  //tft.setFont(&ROBOTECH_GP9pt7b);
  
  for (int i = 0; i < 3; i++) {
    int x = 5 + (i * 80);
    int y = 200;
    
    
    // Vẽ viền nút (màu khác nếu đang được chọn)
    //if (quickTempMode && currentQuickIndex == i) {
      //tft.drawRoundRect(x, y, 50, 25, 5, ST77XX_YELLOW);
    //} else {
    tft.drawRoundRect(x + 20, y + 4, 70, 35, 5, ST77XX_WHITE);
    //}
    
    // Vẽ chữ "Q" và số

    
    // Vẽ giá trị nhiệt độ bên dưới
     tft.setFont(&ROBOTECH_GP20pt7b);
    tft.setCursor(x + 25, y + 30);
    tft.print(quickTemp[i]);
    //tft.print("C");
  }
}

  void updatePID() {
    if(usePlotterInterface || inMenu || currentMode == MODE_SLEEP) return;

    // Xác định nhiệt độ mục tiêu dựa trên chế độ
    int targetTemp = (currentMode == MODE_SLEEP) ? sleepTemperature : setpoint;
    
    // Tính toán giá trị hiện tại
    int pwmValue = myPID.step(targetTemp, rawADC);
    int currentTempDiff = abs(targetTemp - rawADC);
    
    // Kiểm tra có cần cập nhật không
    bool needUpdate = (abs(pwmValue - lastPWM) >= pwmThreshold) || 
                     (abs(currentTempDiff - lastTempDiff) >= tempDiffThreshold);

    if (needUpdate) {
      // Xóa và vẽ khung thanh
      tft.fillRect(285, 65, 30, 165, ST77XX_BLACK);
      tft.drawRoundRect(284, 64, 32, 167, 5, ST77XX_WHITE);
      
      // Tính chiều cao thanh (thay vì độ rộng)
      int meter_height = map(pwmValue, 0, 255, 0, 165);
      
      // Chọn màu dựa trên độ lệch nhiệt độ
      uint16_t bar_color;
      if (currentMode == MODE_SLEEP) {
          bar_color = (currentTempDiff > 20) ? tft.color565(255, 68, 0) : tft.color565(255, 180, 50); 
      } else {
          bar_color = (currentTempDiff > 20) ? tft.color565(255, 68, 0) : tft.color565(255, 180, 50);
      }
      
      // Vẽ thanh dọc từ dưới lên
      int start_y = 230 - meter_height;
      tft.fillRoundRect(285, start_y, 30, meter_height, 5, bar_color);
      
      // Cập nhật giá trị lần cuối
      lastPWM = pwmValue;
      lastTempDiff = currentTempDiff;
    }
  }

  void updateVoltageDisplay() {
   if(usePlotterInterface || inMenu || currentMode == MODE_SLEEP) return;
    // Khung hiển thị điện áp
    tft.setTextColor(ST77XX_WHITE);
    tft.setFont(&ROBOTECH_GP26pt7b);
    tft.drawRoundRect(190, 5, 130, 47, 6, ST77XX_WHITE);
    tft.setCursor(287, 40);
    tft.print("V");
    
    // Chuyển điện áp thành chuỗi
    String voltageText = (voltage < 0.1f) ? "0.0" : String(voltage, 1);
    
    // Tính toán kích thước văn bản
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(voltageText, 0, 0, &x1, &y1, &w, &h);
    
    // Tính vị trí giữa trong vùng 93 pixel (195-287)
    int16_t x = 193 + (95 - w) / 2;
    
    // Xóa và vẽ lại giá trị
    tft.setFont(&ROBOTECH_GP26pt7b);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.fillRect(193, 7, 95, 41, ST77XX_BLACK);
    tft.setCursor(x, 40); // Căn giữa theo chiều ngang
    tft.print(voltageText);
  }

   void updateVoltageDisplay1() {
    if(!usePlotterInterface || inMenu || currentMode == MODE_SLEEP) return;
    // Khung hiển thị điện áp
    tft.setTextColor(ST77XX_WHITE);
    tft.setFont(&ROBOTECH_GP9pt7b);
    tft.setCursor(63, 65);
    tft.print("V");
    tft.setCursor(5, 40);
    tft.print("Voltage");
    
    // Chuyển điện áp thành chuỗi
    String voltageText = (voltage < 0.1f) ? "0.0" : String(voltage, 1);
    
    // Tính toán kích thước văn bản
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(voltageText, 0, 0, &x1, &y1, &w, &h);
    
    // Tính vị trí giữa trong vùng 93 pixel (195-287)
    int16_t x = 2 + (30 - w) / 2;
    
    // Xóa và vẽ lại giá trị
    tft.setFont(&ROBOTECH_GP16pt7b);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.fillRect(2, 50, 58, 16, ST77XX_BLACK);
    tft.setCursor(x, 65); // Căn giữa theo chiều ngang
    tft.print(voltageText);
  }

  
///////////////////////////////////////////////////////////////////////////////

  int readStableTempSensor() {
    int16_t adcValue = analogRead(SENSOR);
    //int rawValue = adcValue / 4;
    
    // Áp dụng bộ lọc Kalman
    K = P / (P + R);
    X = X + K * (adcValue - X);
    P = (1 - K) * P + Q;
    float rawValue = X / 4.0;
    // Thêm bộ lọc EMA (Exponential Moving Average)
    filteredRawADC = filteredRawADC * (1 - FILTER_FACTOR) + rawValue * FILTER_FACTOR;
    // Giới hạn giá trị tối đa là 999
    if (filteredRawADC > 999) {
        filteredRawADC = 999;
    }
    
    return (int)round(filteredRawADC);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////
  void initGraph() {
    if(!usePlotterInterface || currentMode == MODE_SLEEP) return;    

    // Vẽ các đường ngang (chia hàng)
    for (int i = 1; i < 2; i++) {
    int yy = 30 + i * 40;
    tft.drawFastHLine(5, yy, 3*70, tft.color565(80, 80, 80));
    }
  
    // Vẽ các đường dọc (chia cột)
    for (int i = 1; i < 3; i++) {
    int xx = 5 + i * 70;
    tft.drawFastVLine(xx, 30, 2*40, tft.color565(80, 80, 80));
    }
    tft.drawFastHLine(0 , 25, 320, ST77XX_WHITE);
    //tft.fillRect(160, 65, 3, 38, ST77XX_WHITE); // Dày hơn
    //tft.drawRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT, ST77XX_WHITE);
    tft.drawFastVLine(GRAPH_X, GRAPH_Y, GRAPH_HEIGHT - 1, ST77XX_WHITE);
    // Vẽ lưới ngang ngắn (không chạm viền phải)
    //const int H_GRID_LENGTH = GRAPH_WIDTH - 2; // Độ dài lưới ngang
    for (int i = 0; i <= 5; i++) {
        int y = GRAPH_Y + GRAPH_HEIGHT - 1 - (i * (GRAPH_HEIGHT / 5));
        tft.drawFastHLine(GRAPH_X, y, GRAPH_WIDTH, tft.color565(80, 80, 80));
        
        // Hiển thị giá trị trục Y
        tft.setFont(&ROBOTECH_GP9pt7b);
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.setCursor(GRAPH_X - 30, y + 5);
        tft.print(i * 100);
    }

     
        
    
    // Vẽ lưới dọc ngắn (không chạm viền trên/dưới)
    //const int V_GRID_HEIGHT = GRAPH_HEIGHT - 2; // Chiều cao lưới dọc
    for (int x = GRAPH_X + 30; x <= GRAPH_X + GRAPH_WIDTH; x += 30) {
        tft.drawFastVLine(x, GRAPH_Y, GRAPH_HEIGHT, tft.color565(80, 80, 80));
    }
    
    graphInitialized = true;
  }

  void updateGraph() {
    if(!usePlotterInterface || currentMode == MODE_SLEEP || inMenu) return;
    
    static unsigned long lastGraphUpdate = 0;
    const unsigned long graphUpdateInterval = 50; // 20 FPS
    const uint8_t LINE_THICKNESS = 3; // Độ dày đường (3 pixel)
    
    // Giới hạn tốc độ cập nhật đồ thị
    if(millis() - lastGraphUpdate < graphUpdateInterval) return;
    lastGraphUpdate = millis();
    
    if (!graphInitialized) {
        initGraph();
        // Khởi tạo toàn bộ đồ thị với giá trị ban đầu
        for (int i = 0; i < MAX_POINTS; i++) {
            tempHistory[i] = filteredRawADC;
            setpointHistory[i] = setpoint;
            pwmHistory[i] = pwmOutput;
        }
        historyIndex = 0;
        return;
    }
    
    // Lưu giá trị mới
    tempHistory[historyIndex] = rawADC;
    setpointHistory[historyIndex] = setpoint;
    pwmHistory[historyIndex] = pwmOutput;
    
    // Tính toán vị trí x, đảm bảo nằm trong lưới
    int x_pos = constrain(
        GRAPH_X + 1 + map(historyIndex, 0, MAX_POINTS-1, 0, GRAPH_WIDTH-2),
        GRAPH_X + 1, 
        GRAPH_X + GRAPH_WIDTH - 2
    );
    
    // Điều chỉnh cách tính toán tọa độ y để sát cạnh
    int temp_y = GRAPH_Y + GRAPH_HEIGHT - 1 - map(constrain(rawADC, 0, 500), 0, 500, 0, GRAPH_HEIGHT-1);
    int set_y = GRAPH_Y + GRAPH_HEIGHT - 1 - map(constrain(setpoint, 0, 500), 0, 500, 0, GRAPH_HEIGHT-1);
    int pwm_y = GRAPH_Y + GRAPH_HEIGHT - 1 - map(constrain(pwmOutput, 0, 500), 0, 500, 0, GRAPH_HEIGHT-1);
    
    // Vẽ đường nối với độ dày (nét liền)
    if(historyIndex > 0) {
        int prev_x = constrain(
            GRAPH_X + 1 + map(historyIndex-1, 0, MAX_POINTS-1, 0, GRAPH_WIDTH-2),
            GRAPH_X + 1,
            GRAPH_X + GRAPH_WIDTH - 2
        );
        int prev_temp_y = GRAPH_Y + GRAPH_HEIGHT - 1 - map(constrain(tempHistory[historyIndex-1], 0, 500), 0, 500, 0, GRAPH_HEIGHT-1);
        int prev_set_y = GRAPH_Y + GRAPH_HEIGHT - 1 - map(constrain(setpointHistory[historyIndex-1], 0, 500), 0, 500, 0, GRAPH_HEIGHT-1);
        int prev_pwm_y = GRAPH_Y + GRAPH_HEIGHT - 1 - map(constrain(pwmHistory[historyIndex-1], 0, 500), 0, 500, 0, GRAPH_HEIGHT-1);
        
        // Vẽ đường dày với kiểm tra biên
        drawThickLine(prev_x, prev_temp_y, x_pos, temp_y, LINE_THICKNESS, tft.color565(255,255,0));
        drawThickLine(prev_x, prev_set_y, x_pos, set_y, LINE_THICKNESS, tft.color565(0,191,255));
        drawThickLine(prev_x, prev_pwm_y, x_pos, pwm_y, LINE_THICKNESS, ST77XX_GREEN);
    }
    else {
        // Vẽ điểm đầu tiên
        tft.fillCircle(
            constrain(x_pos, GRAPH_X + LINE_THICKNESS, GRAPH_X + GRAPH_WIDTH - LINE_THICKNESS - 1),
            temp_y,
            LINE_THICKNESS/2, 
            ST77XX_CYAN
        );
        tft.fillCircle(
            constrain(x_pos, GRAPH_X + LINE_THICKNESS, GRAPH_X + GRAPH_WIDTH - LINE_THICKNESS - 1),
            set_y,
            LINE_THICKNESS/2, 
            ST77XX_RED
        );
        tft.fillCircle(
            constrain(x_pos, GRAPH_X + LINE_THICKNESS, GRAPH_X + GRAPH_WIDTH - LINE_THICKNESS - 1),
            pwm_y,
            LINE_THICKNESS/2, 
            ST77XX_GREEN
        );
    }
    
    historyIndex = (historyIndex + 1) % MAX_POINTS;
    
    // Xử lý khi quay về đầu đồ thị
    if (historyIndex == 0) {
        tft.fillRect(GRAPH_X + 1, GRAPH_Y , GRAPH_WIDTH , GRAPH_HEIGHT + 1, ST77XX_BLACK);
        initGraph();
    }
  }

  // Hàm vẽ đường dày với kiểm tra biên (đã điều chỉnh)
  void drawThickLine(int x0, int y0, int x1, int y1, uint8_t thickness, uint16_t color) {
    // Đảm bảo tọa độ nằm trong vùng lưới (đã bỏ constrain y để có thể chạm cạnh)
    x0 = constrain(x0, GRAPH_X + thickness, GRAPH_X + GRAPH_WIDTH - thickness - 1);
    x1 = constrain(x1, GRAPH_X + thickness, GRAPH_X + GRAPH_WIDTH - thickness - 1);
    
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        // Vẽ điểm với độ dày (đã điều chỉnh kiểm tra biên y)
        for (int i = -thickness/2; i <= thickness/2; i++) {
            for (int j = -thickness/2; j <= thickness/2; j++) {
                int px = x0 + i;
                int py = y0 + j;
                if (px >= GRAPH_X + 1 && px <= GRAPH_X + GRAPH_WIDTH - 2) {
                    tft.drawPixel(px, py, color);
                }
            }
        }
        
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
  }
  
// CÁC HÀM CÒN LẠI GIỮ NGUYÊN (handleSleepButton, checkIronConnection, drawplotter, drawInterface, ...)

void loop() {

      if (apMode) {
        server.handleClient();
        
        // Tự động thoát AP sau 5 phút
        if (millis() - apStartTime > AP_TIMEOUT) {
            apMode = false;
            WiFi.mode(WIFI_STA);
            // Quay lại menu chính
            inMenu = false;
            currentMode = MODE_GOOT;
            if (usePlotterInterface) {
                drawplotter();
            } else {
                drawInterface();
            }
            return;
        }
    }



  
  // Xử lý menu navigation nếu đang trong menu
  if (inMenu) {
    handleMenuNavigation();
    
    // TẮT GIA NHIỆT KHI Ở TRONG MENU
    analogWrite(pwmPin, 0);
    return; // Thoát khỏi loop() ngay nếu đang trong menu
  }
  
  // Xử lý chính
  handleEncoderButton();
  handleSleepButton();
  checkIronConnection();
  
  if (shouldBlink) {
    displayIronDisconnected();
  }
  
  // Xử lý encoder trong chế độ sleep
  if (currentMode == MODE_SLEEP && encoderPos != 0) {
    sleepTemperature = constrain(sleepTemperature + encoderPos, 100, 250);
    encoderPos = 0;
    setupsleep();
    saveSleepSettings();
  }
  // Xử lý encoder trong chế độ bình thường
  // Xử lý encoder trong chế độ bình thường
  else if (currentMode == MODE_GOOT && encoderPos != 0) {
    setpoint = constrain(setpoint + encoderPos, minTemp, maxTemp); // Sử dụng minTemp và maxTemp
    encoderPos = 0;
    if (usePlotterInterface) {
      updateSetpoint1();
    } else {
      updateSetpoint();
    }
    saveSetpointToEEPROM();
  }

  // Cập nhật điện áp
  static unsigned long lastVoltageUpdate = 0;
  if (millis() - lastVoltageUpdate >= 500) {
    lastVoltageUpdate = millis();
    
    int16_t adc_value = analogRead(VOL); 
    float newVoltage = 0.0f;
    
    if (adc_value > 0) {
      float newVoltage1 = ((adc_value * VOLTAGE_REF) / ADC_RESOLUTION) / VOLTAGE_DIVIDER_RATIO;
      newVoltage = max(0.0f, newVoltage1 + CALIBRATION_OFFSET);
    }
    
    static float lastVoltage = 0.0f;
    if (abs(newVoltage - lastVoltage) >= 0.1f) {
      voltage = newVoltage;
      lastVoltage = voltage;
      if (usePlotterInterface) {
        updateVoltageDisplay1();
      } else {
        updateVoltageDisplay();
      }
    }
  }

  // Cập nhật PID và nhiệt độ
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 50) {
    lastUpdate = millis();
    rawADC = readStableTempSensor();
    
    if (currentMode == MODE_SLEEP) {
      pwmOutput = myPID.step(sleepTemperature, rawADC);
    } else {
      pwmOutput = myPID.step(setpoint, rawADC);
    }
    
    analogWrite(pwmPin, pwmOutput);
    
    if (usePlotterInterface) {
      updateDisplay1();
      updateGraph();
    } else {
      updateDisplay();
      updatePID();
    }
  }

  // Gửi dữ liệu debug
  static unsigned long lastSerialUpdate = 0;
  if (millis() - lastSerialUpdate >= 100) {
    lastSerialUpdate = millis();
    Serial.print("Setpoint:"); Serial.print(setpoint);
    Serial.print(",TEMP:"); Serial.print(rawADC);
    Serial.print(",PWM:"); Serial.println(pwmOutput);
  }
}