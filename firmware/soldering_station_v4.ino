/*
 * Паяльная станция ZM-R5860 - ESP32-S3 Контроллер v4.0
 * 
 * ОБНОВЛЕНИЯ v4.0:
 * - Оптимизированные GPIO пины (без конфликтов SPI)
 * - Автонастройка PID регуляторов
 * - Защита от обрыва термопар
 * - OTA обновления
 * - REST API для внешних приложений
 * - Множественные профили пайки
 * - Звуковые сигналы и кнопки управления
 * 
 * GPIO подключения (ОБНОВЛЕНО):
 * MAX6675 #1 (Top) CS    -> GPIO 5
 * MAX6675 #2 (Bottom) CS -> GPIO 6  
 * MAX6675 #3 (IR) CS     -> GPIO 7
 * MAX6675 #4 (External) CS -> GPIO 8
 * SCK                    -> GPIO 18
 * SO                     -> GPIO 19
 * SSR_TOP                -> GPIO 21
 * SSR_BOTTOM             -> GPIO 47
 * SSR_IR                 -> GPIO 48
 * FAN_OUT                -> GPIO 14
 * STATUS_LED             -> GPIO 2
 * SD_CS                  -> GPIO 10 (SPI2)
 * BUTTON_START           -> GPIO 0
 * BUTTON_STOP            -> GPIO 1
 * BUZZER                 -> GPIO 15
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include "max6675.h"

// -------- Конфигурация GPIO (ОБНОВЛЕНО) --------
#define PIN_MAX_SCK     18
#define PIN_MAX_SO      19
#define PIN_MAX1_CS     5   // Top fan
#define PIN_MAX2_CS     6   // Bottom fan  
#define PIN_MAX3_CS     7   // IR table
#define PIN_MAX4_CS     8   // External TC
#define PIN_SSR_TOP     21
#define PIN_SSR_BOTTOM  47
#define PIN_SSR_IR      48
#define PIN_FAN         14  // Cooling fan
#define PIN_TOP_FAN     16  // Top heating fan (PWM)
#define PIN_BOTTOM_FAN  17  // Bottom heating fan (PWM)
#define PIN_LED         2
#define PIN_SD_CS       10  // SPI2
#define PIN_BUTTON_START 0
#define PIN_BUTTON_STOP  1
#define PIN_BUZZER       15

// -------- Термопары --------
MAX6675 tcTop(PIN_MAX_SCK, PIN_MAX1_CS, PIN_MAX_SO);
MAX6675 tcBottom(PIN_MAX_SCK, PIN_MAX2_CS, PIN_MAX_SO);
MAX6675 tcIR(PIN_MAX_SCK, PIN_MAX3_CS, PIN_MAX_SO);
MAX6675 tcExternal(PIN_MAX_SCK, PIN_MAX4_CS, PIN_MAX_SO);

// -------- WiFi и веб-сервер --------
WebServer server(80);
String wifiMode = "ap";
String wifiSsid = "ZM-R5860";
String wifiPass = "reflow123";
String deviceHostname = "ZM-R5860";
bool wifiConfigured = false;
bool showWifiSetup = true;

// -------- GitHub интеграция --------
const String GITHUB_OWNER = "PavelS2180";
const String GITHUB_REPO = "Soldering-Station";
const String GITHUB_TOKEN = ""; // Добавить токен для приватного репозитория
const String GITHUB_API_URL = "https://api.github.com/repos/" + GITHUB_OWNER + "/" + GITHUB_REPO;
const String FIRMWARE_VERSION = "4.0.0";

// -------- Система логирования --------
bool debugMode = true;
bool githubLogging = true;
String currentLogFile = "";
uint32_t lastLogUpload = 0;
const uint32_t LOG_UPLOAD_INTERVAL = 300000; // 5 минут

// -------- NTP --------
bool timeReady = false;
void setupTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  for (int i = 0; i < 20; i++) {
    time_t now = time(nullptr);
    if (now > 1700000000) {
      timeReady = true;
      break;
    }
    delay(200);
  }
}

// -------- Хранилище данных --------
enum Store { USE_SD, USE_SPIFFS };
Store storeMode = USE_SD;
bool sdOK = false;

fs::FS& activeFS() {
  return (storeMode == USE_SD && sdOK) ? SD : SPIFFS;
}

bool ensureFS() {
  if (storeMode == USE_SD) {
    if (!sdOK) {
      // Инициализация SD с отдельным SPI2
      SPI.begin(12, 13, 11, 10); // SCK, MISO, MOSI, CS
      sdOK = SD.begin(10);
      if (!sdOK) {
        Serial.println("SD Card failed, using SPIFFS");
        SPIFFS.begin(true);
        return false;
      }
    }
    return true;
  } else {
    SPIFFS.begin(true);
    return true;
  }
}

// -------- Профили и фазы пайки --------
struct Phase {
  String name;
  float targetC;
  uint32_t seconds;
  float Kp;
  float Ki;
  bool useTop;
  bool useBottom; 
  bool useIR;
  float topFanSpeed;    // Скорость верхнего фена (0-100%)
  float bottomFanSpeed; // Скорость нижнего фена (0-100%)
};

const int MAX_PHASES = 10;
const int MAX_PROFILES = 20;

struct Preset {
  String name;
  int n;
  float overLimitC;
  Phase phases[MAX_PHASES];
} currentPreset;

// Массив всех профилей
Preset profiles[MAX_PROFILES];
int currentProfileIndex = 0;
int totalProfiles = 0;

enum RunState { IDLE, RUNNING, DONE, ABORTED, AUTOTUNING };
RunState runState = IDLE;

uint8_t currentPhase = 0;
uint32_t phaseStartMs = 0, procStartMs = 0;
float tempTop = 0, tempBottom = 0, tempIR = 0, tempExternal = 0;
float outTop = 0, outBottom = 0, outIR = 0;

// -------- Управление вентиляторами нагрева --------
float topFanSpeed = 0;      // Скорость верхнего фена (0-100%)
float bottomFanSpeed = 0;   // Скорость нижнего фена (0-100%)
bool fansEnabled = false;   // Флажок включения фенов

// -------- PID контроллеры с автонастройкой --------
struct PID {
  float Kp, Ki, Kd, integ, deriv, lastError, outMin = 0, outMax = 100;
  bool enabled = false;
  bool autotuned = false;
  float autotuneTarget = 0;
  uint32_t autotuneStart = 0;
  float autotuneData[100];
  int autotuneIndex = 0;
};
PID pidTop, pidBottom, pidIR;

// -------- Защита от обрыва термопар --------
struct ThermocoupleStatus {
  bool connected = true;
  float lastValidTemp = 25.0;
  uint32_t lastValidTime = 0;
  int errorCount = 0;
};
ThermocoupleStatus tcStatus[4];

// -------- Кнопки управления --------
struct ButtonState {
  bool pressed = false;
  bool lastState = false;
  uint32_t lastPress = 0;
  uint32_t debounceTime = 50;
};
ButtonState startButton, stopButton;

// -------- Управление вентиляторами нагрева --------
void setHeatingFans(float topSpeed = 50, float bottomSpeed = 50) {
  // Фены работают ВСЕГДА, только меняется скорость
  topFanSpeed = constrain(topSpeed, 0, 100);
  bottomFanSpeed = constrain(bottomSpeed, 0, 100);
  
  // PWM управление (0-255)
  analogWrite(PIN_TOP_FAN, (int)(topFanSpeed * 2.55));
  analogWrite(PIN_BOTTOM_FAN, (int)(bottomFanSpeed * 2.55));
  
  Serial.println("Fan speeds set - Top: " + String(topFanSpeed) + "%, Bottom: " + String(bottomFanSpeed) + "%");
}

void setHeatingFansForCooling(float coolingRate = 2.0) {
  // Настройка скорости фенов для контролируемого охлаждения
  // coolingRate: 1.0 = медленное, 3.0 = быстрое охлаждение
  
  if (coolingRate <= 1.5) {
    // Медленное охлаждение - низкая скорость фенов
    setHeatingFans(20, 20);
  } else if (coolingRate <= 2.5) {
    // Умеренное охлаждение - средняя скорость фенов
    setHeatingFans(40, 40);
  } else {
    // Быстрое охлаждение - высокая скорость фенов
    setHeatingFans(60, 60);
  }
  
  Serial.println("Cooling fans set for rate: " + String(coolingRate) + "°C/sec");
}

// -------- Система дебаг логирования --------
void debugLog(String level, String message) {
  if (!debugMode) return;
  
  String timestamp = "";
  if (timeReady) {
    time_t now = time(nullptr);
    struct tm tmnow;
    localtime_r(&now, &tmnow);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmnow);
    timestamp = String(buf);
  } else {
    timestamp = String(millis());
  }
  
  String logEntry = "[" + timestamp + "] [" + level + "] " + message;
  
  // Вывод в Serial
  Serial.println(logEntry);
  
  // Запись в файл
  if (currentLogFile.length() > 0) {
    ensureFS();
    fs::FS &fs = activeFS();
    File logFile = fs.open(currentLogFile, FILE_APPEND);
    if (logFile) {
      logFile.println(logEntry);
      logFile.close();
    }
  }
}

void debugInfo(String message) { debugLog("INFO", message); }
void debugWarn(String message) { debugLog("WARN", message); }
void debugError(String message) { debugLog("ERROR", message); }
void debugDebug(String message) { debugLog("DEBUG", message); }

// -------- WiFi настройки --------
void saveWifiSettings() {
  ensureFS();
  fs::FS &fs = activeFS();
  File file = fs.open("/wifi_config.json", FILE_WRITE);
  
  if (file) {
    JsonDocument doc;
    doc["mode"] = wifiMode;
    doc["ssid"] = wifiSsid;
    doc["password"] = wifiPass;
    doc["hostname"] = deviceHostname;
    doc["configured"] = wifiConfigured;
    
    serializeJson(doc, file);
    file.close();
    debugInfo("WiFi settings saved");
  } else {
    debugError("Failed to save WiFi settings");
  }
}

void loadWifiSettings() {
  ensureFS();
  fs::FS &fs = activeFS();
  
  if (fs.exists("/wifi_config.json")) {
    File file = fs.open("/wifi_config.json", FILE_READ);
    if (file) {
      JsonDocument doc;
      deserializeJson(doc, file);
      
      wifiMode = doc["mode"].as<String>();
      wifiSsid = doc["ssid"].as<String>();
      wifiPass = doc["password"].as<String>();
      deviceHostname = doc["hostname"].as<String>();
      wifiConfigured = doc["configured"].as<bool>();
      
      file.close();
      debugInfo("WiFi settings loaded: " + wifiMode + " - " + wifiSsid);
    }
  } else {
    debugInfo("No WiFi config found, using defaults");
  }
}

// -------- WiFi сканирование --------
String scanWiFiNetworks() {
  debugInfo("Scanning WiFi networks...");
  
  int n = WiFi.scanNetworks();
  if (n == 0) {
    debugWarn("No networks found");
    return "[]";
  }
  
  String networks = "[";
  for (int i = 0; i < n; i++) {
    if (i > 0) networks += ",";
    networks += "{";
    networks += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
    networks += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    networks += "\"encryption\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false");
    networks += "}";
  }
  networks += "]";
  
  debugInfo("Found " + String(n) + " networks");
  return networks;
}

// -------- Автоматическое обновление с GitHub --------
bool checkForUpdates() {
  if (WiFi.status() != WL_CONNECTED) {
    debugWarn("No WiFi connection for update check");
    return false;
  }
  
  HTTPClient http;
  String url = GITHUB_API_URL + "/releases/latest";
  
  http.begin(url);
  if (GITHUB_TOKEN.length() > 0) {
    http.addHeader("Authorization", "token " + GITHUB_TOKEN);
  }
  http.addHeader("User-Agent", "ZM-R5860-Firmware");
  
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    http.end();
    
    JsonDocument doc;
    deserializeJson(doc, payload);
    
    String latestVersion = doc["tag_name"];
    String downloadUrl = doc["assets"][0]["browser_download_url"];
    
    debugInfo("Latest version: " + latestVersion + ", Current: " + FIRMWARE_VERSION);
    
    if (latestVersion != FIRMWARE_VERSION) {
      debugInfo("Update available! Downloading from: " + downloadUrl);
      return downloadAndInstallUpdate(downloadUrl);
    } else {
      debugInfo("Firmware is up to date");
      return false;
    }
  } else {
    debugError("Failed to check for updates. HTTP code: " + String(httpCode));
    http.end();
    return false;
  }
}

bool downloadAndInstallUpdate(String downloadUrl) {
  HTTPClient http;
  http.begin(downloadUrl);
  if (GITHUB_TOKEN.length() > 0) {
    http.addHeader("Authorization", "token " + GITHUB_TOKEN);
  }
  http.addHeader("User-Agent", "ZM-R5860-Firmware");
  
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    debugInfo("Downloading update, size: " + String(contentLength) + " bytes");
    
    if (Update.begin(contentLength)) {
      WiFiClient* stream = http.getStreamPtr();
      size_t written = Update.writeStream(*stream);
      
      if (written == contentLength) {
        debugInfo("Update downloaded successfully");
        if (Update.end()) {
          debugInfo("Update installed successfully, rebooting...");
          beepSequence(5); // Сигнал об успешном обновлении
          delay(1000);
          ESP.restart();
          return true;
        } else {
          debugError("Update installation failed");
          Update.abort();
        }
      } else {
        debugError("Download incomplete. Expected: " + String(contentLength) + ", Got: " + String(written));
        Update.abort();
      }
    } else {
      debugError("Update begin failed");
    }
  } else {
    debugError("Failed to download update. HTTP code: " + String(httpCode));
  }
  
  http.end();
  return false;
}

// -------- Base64 кодирование --------
String base64Encode(String input) {
  const char base64_chars[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  String output = "";
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];
  int len = input.length();
  
  for (int idx = 0; idx < len; idx++) {
    char_array_3[i++] = input.charAt(idx);
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;
      
      for (i = 0; i < 4; i++) {
        output += base64_chars[char_array_4[i]];
      }
      i = 0;
    }
  }
  
  if (i > 0) {
    for (j = i; j < 3; j++) {
      char_array_3[j] = '\0';
    }
    
    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    
    for (j = 0; j < i + 1; j++) {
      output += base64_chars[char_array_4[j]];
    }
    
    while (i++ < 3) {
      output += '=';
    }
  }
  
  return output;
}

// -------- Загрузка логов на GitHub --------
bool uploadLogToGitHub(String logContent, String filename) {
  if (!githubLogging || WiFi.status() != WL_CONNECTED) {
    debugWarn("GitHub logging disabled or no WiFi");
    return false;
  }
  
  HTTPClient http;
  String url = GITHUB_API_URL + "/contents/logs/" + filename;
  
  // Кодируем содержимое в base64
  String encodedContent = base64Encode(logContent);
  
  JsonDocument doc;
  doc["message"] = "Auto-uploaded log: " + filename;
  doc["content"] = encodedContent;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  http.begin(url);
  http.addHeader("Authorization", "token " + GITHUB_TOKEN);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", "ZM-R5860-Firmware");
  
  int httpCode = http.PUT(jsonString);
  if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED) {
    debugInfo("Log uploaded to GitHub: " + filename);
    http.end();
    return true;
  } else {
    debugError("Failed to upload log. HTTP code: " + String(httpCode));
    String response = http.getString();
    debugError("Response: " + response);
    http.end();
    return false;
  }
}

void uploadCurrentLog() {
  if (currentLogFile.length() == 0) return;
  
  ensureFS();
  fs::FS &fs = activeFS();
  File logFile = fs.open(currentLogFile, FILE_READ);
  
  if (logFile) {
    String logContent = logFile.readString();
    logFile.close();
    
    // Извлекаем имя файла из пути
    String filename = currentLogFile;
    int lastSlash = filename.lastIndexOf('/');
    if (lastSlash >= 0) {
      filename = filename.substring(lastSlash + 1);
    }
    
    uploadLogToGitHub(logContent, filename);
  }
}

// -------- Звуковые сигналы --------
void beep(int duration = 200) {
  for (int i = 0; i < duration / 10; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(5);
    digitalWrite(PIN_BUZZER, LOW);
    delay(5);
  }
}

void beepSequence(int count = 3) {
  for (int i = 0; i < count; i++) {
    beep(100);
    delay(100);
  }
}

// -------- Временные константы --------
const uint32_t SAMPLE_MS = 200;
uint32_t lastSample = 0;
const uint32_t SSR_WINDOW = 1000;
uint32_t ssrWindowStart = 0;

// -------- Логирование --------
File logFile;
String lastLogPath;
uint32_t lastLogFlush = 0;

String getTimestampFilename() {
  if (timeReady) {
    time_t now = time(nullptr);
    struct tm tmnow;
    localtime_r(&now, &tmnow);
    char buf[64];
    strftime(buf, sizeof(buf), "/logs/%Y-%m-%d_%H%M.csv", &tmnow);
    return String(buf);
  } else {
    return String("/logs/run_") + String((uint32_t)millis()) + ".csv";
  }
}

void openLog() {
  ensureFS();
  fs::FS &fs = activeFS();
  if (!fs.exists("/logs")) fs.mkdir("/logs");
  
  // Создаем основной лог процесса
  lastLogPath = getTimestampFilename();
  logFile = fs.open(lastLogPath, FILE_WRITE);
  if (logFile) {
    logFile.println("ms,phase,topC,bottomC,irC,externalC,outTop,outBottom,outIR,autotune");
  }
  
  // Создаем дебаг лог
  String debugLogPath = "/logs/debug_" + String((uint32_t)millis()) + ".log";
  currentLogFile = debugLogPath;
  
  debugInfo("=== SOLDERING STATION STARTED ===");
  debugInfo("Firmware version: " + FIRMWARE_VERSION);
  debugInfo("WiFi mode: " + wifiMode);
  debugInfo("Current profile: " + currentPreset.name);
  debugInfo("Debug logging enabled");
  debugInfo("GitHub logging: " + String(githubLogging ? "enabled" : "disabled"));
}

void writeLogLine() {
  if (!logFile) return;
  uint32_t ms = millis() - procStartMs;
  String phase = (runState == RUNNING) ? currentPreset.phases[currentPhase].name : "-";
  String autotune = (runState == AUTOTUNING) ? "YES" : "NO";
  logFile.printf("%u,%s,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f,%.0f,%s\n", 
                 ms, phase.c_str(), tempTop, tempBottom, tempIR, tempExternal, 
                 outTop, outBottom, outIR, autotune.c_str());
  if (millis() - lastLogFlush > 1000) {
    logFile.flush();
    lastLogFlush = millis();
  }
}

void closeLog() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
  
  // Загружаем дебаг лог на GitHub
  if (currentLogFile.length() > 0) {
    debugInfo("=== SOLDERING STATION STOPPED ===");
    debugInfo("Uploading debug log to GitHub...");
    uploadCurrentLog();
    currentLogFile = "";
  }
}

// -------- Проверка термопар --------
bool checkThermocouple(int index, float temperature) {
  ThermocoupleStatus &status = tcStatus[index];
  
  // Проверка на обрыв (температура 0 или очень низкая)
  if (temperature < 5.0 || temperature > 500.0) {
    status.errorCount++;
    if (status.errorCount > 5) {
      status.connected = false;
      return false;
    }
  } else {
    status.connected = true;
    status.errorCount = 0;
    status.lastValidTemp = temperature;
    status.lastValidTime = millis();
  }
  
  return status.connected;
}

// -------- Автонастройка PID --------
void startAutotune(PID &pid, float target, int sensorIndex) {
  pid.autotuned = false;
  pid.autotuneTarget = target;
  pid.autotuneStart = millis();
  pid.autotuneIndex = 0;
  pid.enabled = true;
  runState = AUTOTUNING;
  beepSequence(2);
  Serial.println("Starting PID autotune for sensor " + String(sensorIndex));
}

void processAutotune(PID &pid, float current, float &output) {
  if (!pid.autotuned && millis() - pid.autotuneStart < 60000) { // 60 секунд автонастройки
    // Собираем данные для автонастройки
    if (pid.autotuneIndex < 100) {
      pid.autotuneData[pid.autotuneIndex] = current;
      pid.autotuneIndex++;
    }
    
    // Простая автонастройка (метод Зиглера-Николса)
    float error = pid.autotuneTarget - current;
    if (abs(error) < 2.0) { // Достигли целевой температуры
      // Рассчитываем коэффициенты
      pid.Kp = 0.6 * pid.autotuneTarget / 10.0;
      pid.Ki = 2.0 * pid.Kp / 60.0;
      pid.Kd = pid.Kp * 60.0 / 8.0;
      pid.autotuned = true;
      runState = RUNNING;
      beepSequence(3);
      Serial.println("PID autotune completed: Kp=" + String(pid.Kp) + " Ki=" + String(pid.Ki));
    }
    
    // Временное управление для автонастройки
    output = (error > 0) ? 50.0 : 0.0;
  } else if (!pid.autotuned) {
    // Таймаут автонастройки
    pid.Kp = 2.0;
    pid.Ki = 0.1;
    pid.Kd = 0.0;
    pid.autotuned = true;
    runState = RUNNING;
    Serial.println("PID autotune timeout, using defaults");
  }
}

// -------- Вспомогательные функции --------
float clamp(float v, float min_val, float max_val) {
  return v < min_val ? min_val : (v > max_val ? max_val : v);
}

float stepPID(PID &p, float target, float current, float dt) {
  if (!p.enabled) return 0;
  
  float error = target - current;
  
  // Автонастройка
  if (runState == AUTOTUNING) {
    processAutotune(p, current, p.integ);
    return p.integ;
  }
  
  // Обычный PID
  p.integ = clamp(p.integ + error * p.Ki * dt, p.outMin, p.outMax);
  p.deriv = (error - p.lastError) / dt;
  p.lastError = error;
  
  float output = error * p.Kp + p.integ + p.deriv * p.Kd;
  return clamp(output, p.outMin, p.outMax);
}

void applyPhasePIDCoeffs() {
  if (currentPhase >= currentPreset.n) return;
  
  Phase &phase = currentPreset.phases[currentPhase];
  pidTop.Kp = phase.Kp;
  pidTop.Ki = phase.Ki;
  pidTop.enabled = phase.useTop;
  
  pidBottom.Kp = phase.Kp;
  pidBottom.Ki = phase.Ki;
  pidBottom.enabled = phase.useBottom;
  
  pidIR.Kp = phase.Kp;
  pidIR.Ki = phase.Ki;
  pidIR.enabled = phase.useIR;
  
  // Сброс интегральной части при смене фазы
  pidTop.integ = pidBottom.integ = pidIR.integ = 0;
}

void startProcess() {
  debugInfo("Starting soldering process");
  debugInfo("Profile: " + currentPreset.name);
  debugInfo("Phases: " + String(currentPreset.n));
  
  runState = RUNNING;
  currentPhase = 0;
  procStartMs = phaseStartMs = millis();
  ssrWindowStart = millis();
  applyPhasePIDCoeffs();
  outTop = outBottom = outIR = 0;
  
  // Устанавливаем скорость фенов для первой фазы
  Phase &phase = currentPreset.phases[currentPhase];
  setHeatingFans(phase.topFanSpeed, phase.bottomFanSpeed);
  
  digitalWrite(PIN_LED, HIGH);
  openLog();
  beep();
  
  debugInfo("Process started - Phase 1: " + phase.name);
  debugInfo("Fan speeds - Top: " + String(phase.topFanSpeed) + "%, Bottom: " + String(phase.bottomFanSpeed) + "%");
}

void stopProcess(bool aborted) {
  runState = aborted ? ABORTED : DONE;
  outTop = outBottom = outIR = 0;
  digitalWrite(PIN_SSR_TOP, LOW);
  digitalWrite(PIN_SSR_BOTTOM, LOW);
  digitalWrite(PIN_SSR_IR, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_LED, LOW);
  
  // Фены продолжают работать для охлаждения
  // Устанавливаем скорость для финального охлаждения
  setHeatingFansForCooling(2.0); // Умеренное охлаждение
  
  closeLog();
  
  if (aborted) {
    beepSequence(5);
    Serial.println("Process aborted - fans continue for cooling");
  } else {
    beepSequence(3);
    Serial.println("Process completed - fans continue for cooling");
  }
}

void nextPhase() {
  currentPhase++;
  if (currentPhase >= currentPreset.n) {
    stopProcess(false);
    return;
  }
  phaseStartMs = millis();
  applyPhasePIDCoeffs();
  
  // Управление вентиляторами нагрева и охлаждения
  Phase &phase = currentPreset.phases[currentPhase];
  
  // Проверяем, это фаза охлаждения?
  if (phase.name.indexOf("Cool") >= 0 || phase.name.indexOf("cool") >= 0) {
    // Фаза охлаждения - выключаем нагреватели, настраиваем фены для охлаждения
    digitalWrite(PIN_SSR_TOP, LOW);
    digitalWrite(PIN_SSR_BOTTOM, LOW);
    digitalWrite(PIN_SSR_IR, LOW);
    
    // Включаем вентилятор охлаждения стола
    digitalWrite(PIN_FAN, HIGH);
    
    // Настраиваем фены для контролируемого охлаждения
    setHeatingFansForCooling(2.0); // Умеренное охлаждение
    
    Serial.println("Cooling phase started - heaters OFF, fans + cooling fan ON");
  } else {
    // Обычная фаза - устанавливаем скорость фенов по профилю
    setHeatingFans(phase.topFanSpeed, phase.bottomFanSpeed);
  }
  
  beep();
  Serial.println("Phase " + String(currentPhase + 1) + ": " + phase.name + 
                 " - Top fan: " + String(topFanSpeed) + "%, Bottom fan: " + String(bottomFanSpeed) + "%");
}

// -------- Управление профилями --------
void loadDefaultProfiles() {
  // Lead-Free BGA
  profiles[0].name = "Lead-Free BGA";
  profiles[0].n = 4;
  profiles[0].overLimitC = 280.0;
  profiles[0].phases[0] = {"Preheat", 165, 90, 2.0, 0.08, true, true, true, 30, 30};
  profiles[0].phases[1] = {"Soak", 190, 60, 2.1, 0.09, true, true, true, 40, 40};
  profiles[0].phases[2] = {"Reflow", 255, 30, 2.5, 0.10, true, false, false, 60, 0};
  profiles[0].phases[3] = {"Cool", 100, 90, 1.0, 0.05, false, false, false, 40, 40}; // Фены работают для охлаждения
  
  // Leaded BGA
  profiles[1].name = "Leaded BGA";
  profiles[1].n = 4;
  profiles[1].overLimitC = 260.0;
  profiles[1].phases[0] = {"Preheat", 150, 90, 2.0, 0.08, true, true, true, 30, 30};
  profiles[1].phases[1] = {"Soak", 180, 60, 2.1, 0.09, true, true, true, 40, 40};
  profiles[1].phases[2] = {"Reflow", 240, 30, 2.5, 0.10, true, false, false, 60, 0};
  profiles[1].phases[3] = {"Cool", 100, 90, 1.0, 0.05, false, false, false, 40, 40}; // Фены работают для охлаждения
  
  // SMD Light
  profiles[2].name = "SMD Light";
  profiles[2].n = 3;
  profiles[2].overLimitC = 250.0;
  profiles[2].phases[0] = {"Preheat", 140, 60, 1.8, 0.07, true, true, true, 20, 20};
  profiles[2].phases[1] = {"Reflow", 230, 20, 2.2, 0.09, true, false, false, 50, 0};
  profiles[2].phases[2] = {"Cool", 100, 60, 1.0, 0.05, false, false, false, 30, 30}; // Фены работают для охлаждения
  
  totalProfiles = 3;
  currentProfileIndex = 0;
  currentPreset = profiles[0];
}

void saveProfiles() {
  ensureFS();
  fs::FS &fs = activeFS();
  File f = fs.open("/profiles.json", "w");
  if (f) {
    JsonDocument doc;
    doc["totalProfiles"] = totalProfiles;
    doc["currentProfileIndex"] = currentProfileIndex;
    
    JsonArray profilesArray = doc["profiles"].to<JsonArray>();
    for (int i = 0; i < totalProfiles; i++) {
      JsonObject profile = profilesArray.add<JsonObject>();
      profile["name"] = profiles[i].name;
      profile["overLimitC"] = profiles[i].overLimitC;
      profile["n"] = profiles[i].n;
      
      JsonArray phasesArray = profile["phases"].to<JsonArray>();
      for (int j = 0; j < profiles[i].n; j++) {
        JsonObject phase = phasesArray.add<JsonObject>();
        phase["name"] = profiles[i].phases[j].name;
        phase["targetC"] = profiles[i].phases[j].targetC;
        phase["seconds"] = profiles[i].phases[j].seconds;
        phase["Kp"] = profiles[i].phases[j].Kp;
        phase["Ki"] = profiles[i].phases[j].Ki;
        phase["useTop"] = profiles[i].phases[j].useTop;
        phase["useBottom"] = profiles[i].phases[j].useBottom;
        phase["useIR"] = profiles[i].phases[j].useIR;
      }
    }
    
    serializeJson(doc, f);
    f.close();
  }
}

void loadProfiles() {
  ensureFS();
  fs::FS &fs = activeFS();
  if (fs.exists("/profiles.json")) {
    File f = fs.open("/profiles.json", "r");
    if (f) {
      JsonDocument doc;
      deserializeJson(doc, f);
      f.close();
      
      totalProfiles = doc["totalProfiles"];
      currentProfileIndex = doc["currentProfileIndex"];
      
      JsonArray profilesArray = doc["profiles"];
      for (int i = 0; i < totalProfiles && i < MAX_PROFILES; i++) {
        JsonObject profile = profilesArray[i];
        profiles[i].name = profile["name"].as<String>();
        profiles[i].overLimitC = profile["overLimitC"];
        profiles[i].n = profile["n"];
        
        JsonArray phasesArray = profile["phases"];
        for (int j = 0; j < profiles[i].n && j < MAX_PHASES; j++) {
          JsonObject phase = phasesArray[j];
          profiles[i].phases[j].name = phase["name"].as<String>();
          profiles[i].phases[j].targetC = phase["targetC"];
          profiles[i].phases[j].seconds = phase["seconds"];
          profiles[i].phases[j].Kp = phase["Kp"];
          profiles[i].phases[j].Ki = phase["Ki"];
          profiles[i].phases[j].useTop = phase["useTop"];
          profiles[i].phases[j].useBottom = phase["useBottom"];
          profiles[i].phases[j].useIR = phase["useIR"];
        }
      }
      
      if (currentProfileIndex < totalProfiles) {
        currentPreset = profiles[currentProfileIndex];
      }
    }
  } else {
    loadDefaultProfiles();
  }
}

// -------- Обработка кнопок --------
void updateButtons() {
  // Кнопка старт
  bool startPressed = !digitalRead(PIN_BUTTON_START);
  if (startPressed && !startButton.lastState && millis() - startButton.lastPress > startButton.debounceTime) {
    startButton.pressed = true;
    startButton.lastPress = millis();
    if (runState == IDLE) {
      startProcess();
    }
  }
  startButton.lastState = startPressed;
  
  // Кнопка стоп
  bool stopPressed = !digitalRead(PIN_BUTTON_STOP);
  if (stopPressed && !stopButton.lastState && millis() - stopButton.lastPress > stopButton.debounceTime) {
    stopButton.pressed = true;
    stopButton.lastPress = millis();
    if (runState == RUNNING || runState == AUTOTUNING) {
      stopProcess(true);
    }
  }
  stopButton.lastState = stopPressed;
}

// -------- HTML интерфейс --------
const char* WIFI_SETUP_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ZM-R5860 - Настройка WiFi</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 20px;
        }
        .container {
            background: white;
            border-radius: 20px;
            box-shadow: 0 20px 40px rgba(0,0,0,0.1);
            padding: 40px;
            max-width: 500px;
            width: 100%;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .logo {
            font-size: 2.5em;
            font-weight: bold;
            color: #333;
            margin-bottom: 10px;
        }
        .subtitle {
            color: #666;
            font-size: 1.1em;
        }
        .mode-selector {
            margin-bottom: 30px;
        }
        .mode-option {
            display: flex;
            align-items: center;
            padding: 15px;
            margin: 10px 0;
            border: 2px solid #e0e0e0;
            border-radius: 10px;
            cursor: pointer;
            transition: all 0.3s ease;
        }
        .mode-option:hover {
            border-color: #667eea;
            background: #f8f9ff;
        }
        .mode-option.selected {
            border-color: #667eea;
            background: #f0f4ff;
        }
        .mode-option input[type="radio"] {
            margin-right: 15px;
            transform: scale(1.2);
        }
        .mode-info {
            flex: 1;
        }
        .mode-title {
            font-weight: bold;
            color: #333;
            margin-bottom: 5px;
        }
        .mode-desc {
            color: #666;
            font-size: 0.9em;
        }
        .wifi-form {
            display: none;
            margin-top: 20px;
        }
        .wifi-form.active {
            display: block;
        }
        .form-group {
            margin-bottom: 20px;
        }
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: bold;
            color: #333;
        }
        .form-group input, .form-group select {
            width: 100%;
            padding: 12px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s ease;
        }
        .form-group input:focus, .form-group select:focus {
            outline: none;
            border-color: #667eea;
        }
        .btn {
            width: 100%;
            padding: 15px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            border-radius: 10px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: transform 0.2s ease;
        }
        .btn:hover {
            transform: translateY(-2px);
        }
        .btn:active {
            transform: translateY(0);
        }
        .btn-secondary {
            background: #6c757d;
            margin-top: 10px;
        }
        .status {
            margin-top: 20px;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
            font-weight: bold;
        }
        .status.success {
            background: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .status.error {
            background: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        .status.info {
            background: #d1ecf1;
            color: #0c5460;
            border: 1px solid #bee5eb;
        }
        .scan-btn {
            background: #28a745;
            margin-bottom: 15px;
        }
        .network-list {
            max-height: 200px;
            overflow-y: auto;
            border: 1px solid #e0e0e0;
            border-radius: 8px;
            margin-bottom: 15px;
        }
        .network-item {
            padding: 10px;
            border-bottom: 1px solid #e0e0e0;
            cursor: pointer;
            transition: background 0.2s ease;
        }
        .network-item:hover {
            background: #f8f9ff;
        }
        .network-item:last-child {
            border-bottom: none;
        }
        .network-name {
            font-weight: bold;
            color: #333;
        }
        .network-signal {
            color: #666;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="logo">🔧 ZM-R5860</div>
            <div class="subtitle">Паяльная станция - Настройка подключения</div>
        </div>

        <div class="mode-selector">
            <div class="mode-option" onclick="selectMode('wifi')">
                <input type="radio" name="mode" value="wifi" id="wifi-mode">
                <div class="mode-info">
                    <div class="mode-title">🌐 Подключение к WiFi</div>
                    <div class="mode-desc">Подключиться к существующей сети WiFi для удаленного управления</div>
                </div>
            </div>
            
            <div class="mode-option" onclick="selectMode('ap')">
                <input type="radio" name="mode" value="ap" id="ap-mode" checked>
                <div class="mode-info">
                    <div class="mode-title">📡 Режим точки доступа</div>
                    <div class="mode-desc">Создать собственную сеть для прямого подключения</div>
                </div>
            </div>
        </div>

        <div class="wifi-form" id="wifi-form">
            <button class="btn scan-btn" onclick="scanNetworks()">🔍 Сканировать сети</button>
            
            <div class="network-list" id="network-list" style="display: none;">
                <!-- Список сетей будет загружен здесь -->
            </div>

            <div class="form-group">
                <label for="ssid">Имя сети (SSID):</label>
                <input type="text" id="ssid" placeholder="Введите имя WiFi сети">
            </div>

            <div class="form-group">
                <label for="password">Пароль:</label>
                <input type="password" id="password" placeholder="Введите пароль WiFi">
            </div>

            <div class="form-group">
                <label for="hostname">Имя устройства в сети:</label>
                <input type="text" id="hostname" value="ZM-R5860" placeholder="Имя для доступа по сети">
            </div>
        </div>

        <div class="wifi-form active" id="ap-form">
            <div class="form-group">
                <label for="ap-ssid">Имя точки доступа:</label>
                <input type="text" id="ap-ssid" value="ZM-R5860" placeholder="Имя сети">
            </div>

            <div class="form-group">
                <label for="ap-password">Пароль:</label>
                <input type="password" id="ap-password" value="reflow123" placeholder="Пароль для подключения">
            </div>

            <div class="form-group">
                <label for="ap-hostname">Имя устройства:</label>
                <input type="text" id="ap-hostname" value="ZM-R5860" placeholder="Имя устройства">
            </div>
        </div>

        <button class="btn" onclick="saveSettings()">💾 Сохранить настройки</button>
        <button class="btn btn-secondary" onclick="skipSetup()">⏭️ Пропустить настройку</button>

        <div id="status"></div>
    </div>

    <script>
        let selectedMode = 'ap';
        let networks = [];

        function selectMode(mode) {
            selectedMode = mode;
            
            // Обновляем визуальное выделение
            document.querySelectorAll('.mode-option').forEach(option => {
                option.classList.remove('selected');
            });
            event.currentTarget.classList.add('selected');
            
            // Показываем/скрываем формы
            document.getElementById('wifi-form').classList.toggle('active', mode === 'wifi');
            document.getElementById('ap-form').classList.toggle('active', mode === 'ap');
            
            // Обновляем радиокнопки
            document.getElementById('wifi-mode').checked = mode === 'wifi';
            document.getElementById('ap-mode').checked = mode === 'ap';
        }

        function scanNetworks() {
            showStatus('Сканирование сетей...', 'info');
            
            fetch('/api/scan-wifi')
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        networks = data.networks;
                        displayNetworks();
                        showStatus(`Найдено ${networks.length} сетей`, 'success');
                    } else {
                        showStatus('Ошибка сканирования: ' + data.error, 'error');
                    }
                })
                .catch(error => {
                    showStatus('Ошибка подключения: ' + error, 'error');
                });
        }

        function displayNetworks() {
            const networkList = document.getElementById('network-list');
            networkList.style.display = 'block';
            networkList.innerHTML = '';

            networks.forEach(network => {
                const item = document.createElement('div');
                item.className = 'network-item';
                item.onclick = () => selectNetwork(network.ssid);
                
                item.innerHTML = `
                    <div class="network-name">${network.ssid}</div>
                    <div class="network-signal">Сигнал: ${network.rssi} dBm ${network.encryption ? '🔒' : '🔓'}</div>
                `;
                
                networkList.appendChild(item);
            });
        }

        function selectNetwork(ssid) {
            document.getElementById('ssid').value = ssid;
            showStatus(`Выбрана сеть: ${ssid}`, 'info');
        }

        function saveSettings() {
            const settings = {
                mode: selectedMode
            };

            if (selectedMode === 'wifi') {
                settings.ssid = document.getElementById('ssid').value;
                settings.password = document.getElementById('password').value;
                settings.hostname = document.getElementById('hostname').value;
                
                if (!settings.ssid) {
                    showStatus('Введите имя сети WiFi', 'error');
                    return;
                }
            } else {
                settings.ap_ssid = document.getElementById('ap-ssid').value;
                settings.ap_password = document.getElementById('ap-password').value;
                settings.ap_hostname = document.getElementById('ap-hostname').value;
            }

            showStatus('Сохранение настроек...', 'info');

            fetch('/api/wifi-config', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(settings)
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showStatus('Настройки сохранены! Перезагрузка...', 'success');
                    setTimeout(() => {
                        window.location.href = '/';
                    }, 2000);
                } else {
                    showStatus('Ошибка сохранения: ' + data.error, 'error');
                }
            })
            .catch(error => {
                showStatus('Ошибка подключения: ' + error, 'error');
            });
        }

        function skipSetup() {
            showStatus('Переход к управлению...', 'info');
            setTimeout(() => {
                window.location.href = '/';
            }, 1000);
        }

        function showStatus(message, type) {
            const status = document.getElementById('status');
            status.textContent = message;
            status.className = `status ${type}`;
            status.style.display = 'block';
        }

        // Инициализация
        document.addEventListener('DOMContentLoaded', function() {
            // Выбираем режим AP по умолчанию
            selectMode('ap');
        });
    </script>
</body>
</html>
)HTML";

const char* INDEX_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ZM-R5860 v4.0</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { font-family: system-ui, Segoe UI, Arial; margin: 16px; max-width: 1200px; }
        .card { border: 1px solid #ccc; border-radius: 10px; padding: 16px; margin: 10px 0; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 8px; }
        input, button, select { padding: 8px; font-size: 14px; margin: 4px; }
        .badge { padding: 4px 8px; border-radius: 999px; background: #eee; margin-left: 6px; }
        .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
        .temp-display { font-size: 18px; font-weight: bold; color: #333; }
        .status { padding: 8px; border-radius: 4px; margin: 4px 0; }
        .status.running { background: #d4edda; color: #155724; }
        .status.idle { background: #f8f9fa; color: #6c757d; }
        .status.error { background: #f8d7da; color: #721c24; }
        .status.autotuning { background: #fff3cd; color: #856404; }
        #temperatureChart { height: 300px; }
        .error { color: red; font-weight: bold; }
    </style>
</head>
<body>
    <h1>Паяльная станция ZM-R5860 v4.0 <span id="state" class="badge">IDLE</span></h1>

    <div class="card">
        <h3>Температуры</h3>
        <div class="grid">
            <div>Верхний фен: <span id="tempTop" class="temp-display">--.-</span> °C <span id="tc1Status"></span></div>
            <div>Нижний фен: <span id="tempBottom" class="temp-display">--.-</span> °C <span id="tc2Status"></span></div>
            <div>IR-стол: <span id="tempIR" class="temp-display">--.-</span> °C <span id="tc3Status"></span></div>
            <div>Внешняя ТС: <span id="tempExternal" class="temp-display">--.-</span> °C <span id="tc4Status"></span></div>
        </div>
        <div class="row">
            <div>Фаза: <b id="phaseName">-</b></div>
            <div>Осталось: <b id="remainTime">-</b> сек</div>
            <div>Выходы: <b id="outputs">0/0/0</b>%</div>
            <div>Фены: <b id="heatingFans">ВЫКЛ</b></div>
            <div>Охлаждение: <b id="fanStatus">ВЫКЛ</b></div>
            <div>Профиль: <b id="currentProfile">-</b></div>
        </div>
    </div>

    <div class="card">
        <h3>График температур</h3>
        <div id="temperatureChart"></div>
    </div>

    <div class="card">
        <h3>Управление профилями</h3>
        <div class="row">
            <label>Профиль: <select id="profileSelect"></select></label>
            <button onclick="addProfile()">+ Новый профиль</button>
            <button onclick="deleteProfile()">- Удалить</button>
            <button onclick="saveProfiles()">Сохранить все</button>
        </div>
        <div id="profileEditor"></div>
    </div>

    <div class="card">
        <h3>Управление</h3>
        <div class="row">
            <button onclick="startProcess()" id="startBtn">Старт</button>
            <button onclick="stopProcess()" id="stopBtn">Стоп</button>
            <button onclick="toggleFan()" id="fanBtn">Вентилятор</button>
            <button onclick="startAutotune()" id="autotuneBtn">Автонастройка PID</button>
            <button onclick="window.location.href='/setup'" id="wifiBtn">🌐 WiFi</button>
            <a id="downloadLog" href="#" download style="display:none;">Скачать лог</a>
        </div>
    </div>

    <div class="card">
        <h3>Настройки</h3>
        <div class="row">
            <label>WiFi режим: <select id="wifiMode"><option value="ap">AP</option><option value="sta">STA</option></select></label>
            <input id="wifiSsid" placeholder="SSID">
            <input id="wifiPass" placeholder="Пароль">
            <button onclick="saveSettings()">Сохранить</button>
        </div>
        <div class="row">
            <label>Хранилище: <select id="storageMode"><option value="sd">SD</option><option value="spiffs">SPIFFS</option></select></label>
            <label>Лимит температуры: <input id="tempLimit" type="number" value="280">°C</label>
        </div>
    </div>

    <script>
        let temperatureData = { time: [], top: [], bottom: [], ir: [], external: [] };
        let maxDataPoints = 100;
        let currentProfiles = [];

        function updateTemperatures() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    // Обновляем температуры
                    document.getElementById('tempTop').textContent = data.temperatures.top.toFixed(1);
                    document.getElementById('tempBottom').textContent = data.temperatures.bottom.toFixed(1);
                    document.getElementById('tempIR').textContent = data.temperatures.ir.toFixed(1);
                    document.getElementById('tempExternal').textContent = data.temperatures.external.toFixed(1);
                    
                    // Статус термопар
                    updateThermocoupleStatus('tc1Status', data.thermocouples[0]);
                    updateThermocoupleStatus('tc2Status', data.thermocouples[1]);
                    updateThermocoupleStatus('tc3Status', data.thermocouples[2]);
                    updateThermocoupleStatus('tc4Status', data.thermocouples[3]);
                    
                    // Обновляем статус
                    document.getElementById('phaseName').textContent = data.phase;
                    document.getElementById('remainTime').textContent = data.remain;
                    document.getElementById('outputs').textContent = data.outputs.top + '/' + data.outputs.bottom + '/' + data.outputs.ir;
                    document.getElementById('heatingFans').textContent = data.heatingFans || 'ВЫКЛ';
                    document.getElementById('fanStatus').textContent = data.fanStatus || 'ВЫКЛ';
                    document.getElementById('currentProfile').textContent = data.currentProfile;
                    document.getElementById('state').textContent = data.state;

                    // Обновляем график
                    updateChart(data.temperatures);
                })
                .catch(error => console.error('Error:', error));
        }

        function updateThermocoupleStatus(elementId, status) {
            const element = document.getElementById(elementId);
            if (status.connected) {
                element.textContent = '✓';
                element.className = '';
            } else {
                element.textContent = '✗';
                element.className = 'error';
            }
        }

        function updateChart(temps) {
            const now = Date.now();
            temperatureData.time.push(now);
            temperatureData.top.push(temps.top);
            temperatureData.bottom.push(temps.bottom);
            temperatureData.ir.push(temps.ir);
            temperatureData.external.push(temps.external);

            if (temperatureData.time.length > maxDataPoints) {
                temperatureData.time.shift();
                temperatureData.top.shift();
                temperatureData.bottom.shift();
                temperatureData.ir.shift();
                temperatureData.external.shift();
            }

            Plotly.newPlot('temperatureChart', [
                { x: temperatureData.time, y: temperatureData.top, name: 'Верхний фен', line: { color: 'red' }},
                { x: temperatureData.time, y: temperatureData.bottom, name: 'Нижний фен', line: { color: 'blue' }},
                { x: temperatureData.time, y: temperatureData.ir, name: 'IR-стол', line: { color: 'orange' }},
                { x: temperatureData.time, y: temperatureData.external, name: 'Внешняя ТС', line: { color: 'green' }}
            ], {
                title: 'Температуры в реальном времени',
                xaxis: { title: 'Время' },
                yaxis: { title: 'Температура (°C)' }
            });
        }

        function startProcess() {
            fetch('/api/start', { method: 'POST' });
        }

        function stopProcess() {
            fetch('/api/stop', { method: 'POST' });
        }

        function toggleFan() {
            fetch('/api/fan', { method: 'POST' });
        }

        function startAutotune() {
            fetch('/api/autotune', { method: 'POST' });
        }

        function loadProfiles() {
            fetch('/api/profiles')
                .then(response => response.json())
                .then(data => {
                    currentProfiles = data.profiles;
                    updateProfileSelect();
                });
        }

        function updateProfileSelect() {
            const select = document.getElementById('profileSelect');
            select.innerHTML = '';
            currentProfiles.forEach((profile, index) => {
                const option = document.createElement('option');
                option.value = index;
                option.textContent = profile.name;
                select.appendChild(option);
            });
        }

        function saveProfiles() {
            fetch('/api/profiles', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ profiles: currentProfiles })
            });
        }

        // Обновляем данные каждые 300мс
        setInterval(updateTemperatures, 300);
        updateTemperatures();
        loadProfiles();
    </script>
</body>
</html>
)HTML";

// -------- REST API обработчики --------
void handleApiStatus() {
  JsonDocument doc;
  doc["state"] = (runState == IDLE) ? "IDLE" : (runState == RUNNING) ? "RUNNING" : (runState == DONE) ? "DONE" : (runState == ABORTED) ? "ABORTED" : "AUTOTUNING";
  doc["phase"] = (runState == RUNNING) ? currentPreset.phases[currentPhase].name : "-";
  doc["currentProfile"] = currentPreset.name;
  
  uint32_t remain = 0;
  if (runState == RUNNING && currentPhase < currentPreset.n) {
    uint32_t elapsed = (millis() - phaseStartMs) / 1000;
    uint32_t duration = currentPreset.phases[currentPhase].seconds;
    remain = (elapsed >= duration) ? 0 : (duration - elapsed);
  }
  doc["remain"] = remain;
  
  JsonObject temps = doc["temperatures"].to<JsonObject>();
  temps["top"] = tempTop;
  temps["bottom"] = tempBottom;
  temps["ir"] = tempIR;
  temps["external"] = tempExternal;
  
  JsonObject outputs = doc["outputs"].to<JsonObject>();
  outputs["top"] = outTop;
  outputs["bottom"] = outBottom;
  outputs["ir"] = outIR;
  
  // Состояние вентиляторов
  doc["fanStatus"] = digitalRead(PIN_FAN) ? "ВКЛ" : "ВЫКЛ";
  
  // Состояние фенов нагрева - фены работают всегда
  doc["heatingFans"] = "ВКЛ (" + String((int)topFanSpeed) + "/" + String((int)bottomFanSpeed) + "%)";
  
  JsonArray thermocouples = doc["thermocouples"].to<JsonArray>();
  for (int i = 0; i < 4; i++) {
    JsonObject tc = thermocouples.add<JsonObject>();
    tc["connected"] = tcStatus[i].connected;
    tc["lastValidTemp"] = tcStatus[i].lastValidTemp;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiStart() {
  if (runState != RUNNING && runState != AUTOTUNING) {
    startProcess();
    server.send(200, "application/json", "{\"status\":\"started\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"already running\"}");
  }
}

void handleApiStop() {
  stopProcess(true);
  server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

void handleApiFan() {
  static bool fanOn = false;
  fanOn = !fanOn;
  digitalWrite(PIN_FAN, fanOn ? HIGH : LOW);
  
  // Логирование состояния вентилятора
  if (fanOn) {
    Serial.println("Cooling fan manually activated");
  } else {
    Serial.println("Cooling fan manually deactivated");
  }
  
  server.send(200, "application/json", fanOn ? "{\"status\":\"fan on\"}" : "{\"status\":\"fan off\"}");
}

void handleApiAutotune() {
  if (runState == IDLE) {
    startAutotune(pidTop, 200.0, 0);
    server.send(200, "application/json", "{\"status\":\"autotune started\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"not idle\"}");
  }
}

void handleApiProfiles() {
  if (server.method() == HTTP_GET) {
    JsonDocument doc;
    JsonArray profilesArray = doc["profiles"].to<JsonArray>();
    
    for (int i = 0; i < totalProfiles; i++) {
      JsonObject profile = profilesArray.add<JsonObject>();
      profile["name"] = profiles[i].name;
      profile["overLimitC"] = profiles[i].overLimitC;
      profile["n"] = profiles[i].n;
      
      JsonArray phasesArray = profile["phases"].to<JsonArray>();
      for (int j = 0; j < profiles[i].n; j++) {
        JsonObject phase = phasesArray.add<JsonObject>();
        phase["name"] = profiles[i].phases[j].name;
        phase["targetC"] = profiles[i].phases[j].targetC;
        phase["seconds"] = profiles[i].phases[j].seconds;
        phase["Kp"] = profiles[i].phases[j].Kp;
        phase["Ki"] = profiles[i].phases[j].Ki;
        phase["useTop"] = profiles[i].phases[j].useTop;
        phase["useBottom"] = profiles[i].phases[j].useBottom;
        phase["useIR"] = profiles[i].phases[j].useIR;
      }
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  } else if (server.method() == HTTP_POST) {
    // Сохранение профилей
    String body = server.arg("plain");
    JsonDocument doc;
    deserializeJson(doc, body);
    
    JsonArray profilesArray = doc["profiles"];
    totalProfiles = 0;
    
    for (JsonObject profile : profilesArray) {
      if (totalProfiles >= MAX_PROFILES) break;
      
      profiles[totalProfiles].name = profile["name"].as<String>();
      profiles[totalProfiles].overLimitC = profile["overLimitC"];
      profiles[totalProfiles].n = profile["n"];
      
      JsonArray phasesArray = profile["phases"];
      for (int j = 0; j < profiles[totalProfiles].n && j < MAX_PHASES; j++) {
        JsonObject phase = phasesArray[j];
        profiles[totalProfiles].phases[j].name = phase["name"].as<String>();
        profiles[totalProfiles].phases[j].targetC = phase["targetC"];
        profiles[totalProfiles].phases[j].seconds = phase["seconds"];
        profiles[totalProfiles].phases[j].Kp = phase["Kp"];
        profiles[totalProfiles].phases[j].Ki = phase["Ki"];
        profiles[totalProfiles].phases[j].useTop = phase["useTop"];
        profiles[totalProfiles].phases[j].useBottom = phase["useBottom"];
        profiles[totalProfiles].phases[j].useIR = phase["useIR"];
      }
      totalProfiles++;
    }
    
    saveProfiles();
    server.send(200, "application/json", "{\"status\":\"saved\"}");
  }
}

void handleApiCheckUpdates() {
  debugInfo("Manual update check requested");
  bool updateAvailable = checkForUpdates();
  
  JsonDocument doc;
  doc["updateAvailable"] = updateAvailable;
  doc["currentVersion"] = FIRMWARE_VERSION;
  doc["status"] = updateAvailable ? "Update downloaded and installed" : "No updates available";
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiUploadLogs() {
  debugInfo("Manual log upload requested");
  uploadCurrentLog();
  
  JsonDocument doc;
  doc["status"] = "Logs uploaded to GitHub";
  doc["logFile"] = currentLogFile;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiDebugMode() {
  String body = server.arg("plain");
  JsonDocument doc;
  deserializeJson(doc, body);
  
  bool newDebugMode = doc["debugMode"];
  bool newGithubLogging = doc["githubLogging"];
  
  debugMode = newDebugMode;
  githubLogging = newGithubLogging;
  
  debugInfo("Debug mode: " + String(debugMode ? "enabled" : "disabled"));
  debugInfo("GitHub logging: " + String(githubLogging ? "enabled" : "disabled"));
  
  JsonDocument response;
  response["debugMode"] = debugMode;
  response["githubLogging"] = githubLogging;
  response["status"] = "Settings updated";
  
  String responseStr;
  serializeJson(response, responseStr);
  server.send(200, "application/json", responseStr);
}

void handleApiScanWifi() {
  debugInfo("WiFi scan requested");
  String networks = scanWiFiNetworks();
  
  JsonDocument doc;
  doc["success"] = true;
  doc["networks"] = networks;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleApiWifiConfig() {
  String body = server.arg("plain");
  JsonDocument doc;
  deserializeJson(doc, body);
  
  String mode = doc["mode"].as<String>();
  
  if (mode == "wifi") {
    wifiMode = "wifi";
    wifiSsid = doc["ssid"].as<String>();
    wifiPass = doc["password"].as<String>();
    deviceHostname = doc["hostname"].as<String>();
    
    debugInfo("WiFi config: " + wifiSsid + " - " + deviceHostname);
    
    // Попытка подключения к WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    
    // Ждем подключения до 10 секунд
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500);
      attempts++;
      debugInfo("Connecting to WiFi... " + String(attempts));
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      wifiConfigured = true;
      showWifiSetup = false;
      saveWifiSettings();
      
      JsonDocument response;
      response["success"] = true;
      response["message"] = "WiFi connected successfully";
      response["ip"] = WiFi.localIP().toString();
      
      String responseStr;
      serializeJson(response, responseStr);
      server.send(200, "application/json", responseStr);
      
      debugInfo("WiFi connected: " + WiFi.localIP().toString());
    } else {
      // Возвращаемся в режим AP
      WiFi.mode(WIFI_AP);
      WiFi.softAP(wifiSsid.c_str(), wifiPass.c_str());
      
      JsonDocument response;
      response["success"] = false;
      response["error"] = "Failed to connect to WiFi network";
      
      String responseStr;
      serializeJson(response, responseStr);
      server.send(200, "application/json", responseStr);
      
      debugError("Failed to connect to WiFi");
    }
  } else {
    // Режим AP
    wifiMode = "ap";
    wifiSsid = doc["ap_ssid"].as<String>();
    wifiPass = doc["ap_password"].as<String>();
    deviceHostname = doc["ap_hostname"].as<String>();
    
    debugInfo("AP config: " + wifiSsid + " - " + deviceHostname);
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifiSsid.c_str(), wifiPass.c_str());
    
    wifiConfigured = true;
    showWifiSetup = false;
    saveWifiSettings();
    
    JsonDocument response;
    response["success"] = true;
    response["message"] = "AP mode configured";
    response["ip"] = WiFi.softAPIP().toString();
    
    String responseStr;
    serializeJson(response, responseStr);
    server.send(200, "application/json", responseStr);
    
    debugInfo("AP mode configured: " + WiFi.softAPIP().toString());
  }
}

// -------- OTA обновления --------
void setupOTA() {
  ArduinoOTA.setHostname("ZM-R5860");
  ArduinoOTA.setPassword("reflow123");
  
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Start updating " + type);
    digitalWrite(PIN_LED, HIGH);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    digitalWrite(PIN_LED, LOW);
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
}

// -------- Настройка --------
void setup() {
  Serial.begin(115200);
  
  // Настройка GPIO
  pinMode(PIN_SSR_TOP, OUTPUT);
  pinMode(PIN_SSR_BOTTOM, OUTPUT);
  pinMode(PIN_SSR_IR, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_TOP_FAN, OUTPUT);
  pinMode(PIN_BOTTOM_FAN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON_START, INPUT_PULLUP);
  pinMode(PIN_BUTTON_STOP, INPUT_PULLUP);
  
  // Инициализация всех выходов в выключенное состояние
  digitalWrite(PIN_SSR_TOP, LOW);
  digitalWrite(PIN_SSR_BOTTOM, LOW);
  digitalWrite(PIN_SSR_IR, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_TOP_FAN, LOW);
  digitalWrite(PIN_BOTTOM_FAN, LOW);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  
  // Инициализация PWM для фенов нагрева - фены работают всегда
  setHeatingFans(30, 30); // 30% скорость по умолчанию в режиме ожидания

  // Инициализация файловой системы
  SPIFFS.begin(true);
  ensureFS();
  
  // Загрузка WiFi настроек
  loadWifiSettings();
  
  // Загрузка профилей
  loadProfiles();

  // Настройка WiFi
  if (wifiMode == "wifi" && wifiConfigured) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    uint32_t timeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - timeout < 10000) {
      delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
      setupTime();
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
      showWifiSetup = false;
    } else {
      WiFi.mode(WIFI_AP);
      WiFi.softAP(wifiSsid.c_str(), wifiPass.c_str());
      Serial.println("AP mode: " + WiFi.softAPIP().toString());
      showWifiSetup = true;
    }
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifiSsid.c_str(), wifiPass.c_str());
    Serial.println("AP mode: " + WiFi.softAPIP().toString());
    showWifiSetup = true;
  }

  // Настройка OTA
  setupOTA();

  // Настройка веб-сервера
  server.on("/", []() { 
    if (showWifiSetup) {
      server.send(200, "text/html", WIFI_SETUP_HTML);
    } else {
      server.send(200, "text/html", INDEX_HTML);
    }
  });
  server.on("/api/status", handleApiStatus);
  server.on("/api/start", HTTP_POST, handleApiStart);
  server.on("/api/stop", HTTP_POST, handleApiStop);
  server.on("/api/fan", HTTP_POST, handleApiFan);
  server.on("/api/autotune", HTTP_POST, handleApiAutotune);
  server.on("/api/profiles", HTTP_GET, handleApiProfiles);
  server.on("/api/profiles", HTTP_POST, handleApiProfiles);
  server.on("/api/check-updates", HTTP_POST, handleApiCheckUpdates);
  server.on("/api/upload-logs", HTTP_POST, handleApiUploadLogs);
  server.on("/api/debug-mode", HTTP_POST, handleApiDebugMode);
  server.on("/api/scan-wifi", HTTP_GET, handleApiScanWifi);
  server.on("/api/wifi-config", HTTP_POST, handleApiWifiConfig);
  server.on("/setup", []() { server.send(200, "text/html", WIFI_SETUP_HTML); });
  
  server.begin();
  Serial.println("Web server started");
  beepSequence(2);
}

// -------- Основной цикл --------
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  updateButtons();
  
  // Автоматическая проверка обновлений каждые 30 минут
  static uint32_t lastUpdateCheck = 0;
  if (millis() - lastUpdateCheck > 1800000) { // 30 минут
    lastUpdateCheck = millis();
    if (WiFi.status() == WL_CONNECTED) {
      debugInfo("Checking for automatic updates...");
      checkForUpdates();
    }
  }
  
  // Автоматическая загрузка логов каждые 5 минут
  if (millis() - lastLogUpload > LOG_UPLOAD_INTERVAL) {
    lastLogUpload = millis();
    if (githubLogging && currentLogFile.length() > 0) {
      debugInfo("Auto-uploading logs to GitHub...");
      uploadCurrentLog();
    }
  }
  
  uint32_t now = millis();

  // Чтение температур
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    tempTop = tcTop.readCelsius();
    tempBottom = tcBottom.readCelsius();
    tempIR = tcIR.readCelsius();
    tempExternal = tcExternal.readCelsius();
    
    // Проверка термопар
    bool tc1OK = checkThermocouple(0, tempTop);
    bool tc2OK = checkThermocouple(1, tempBottom);
    bool tc3OK = checkThermocouple(2, tempIR);
    bool tc4OK = checkThermocouple(3, tempExternal);
    
    // Дебаг логирование температур каждые 10 секунд
    static uint32_t lastTempLog = 0;
    if (now - lastTempLog > 10000) {
      lastTempLog = now;
      debugDebug("Temps - Top:" + String(tempTop, 1) + " Bottom:" + String(tempBottom, 1) + 
                 " IR:" + String(tempIR, 1) + " Ext:" + String(tempExternal, 1));
      debugDebug("TC Status - T1:" + String(tc1OK ? "OK" : "ERR") + " T2:" + String(tc2OK ? "OK" : "ERR") + 
                 " T3:" + String(tc3OK ? "OK" : "ERR") + " T4:" + String(tc4OK ? "OK" : "ERR"));
    }
  }

  // Обработка процесса пайки
  if (runState == RUNNING || runState == AUTOTUNING) {
    // Проверка безопасности
    float maxTemp = max(max(tempTop, tempBottom), max(tempIR, tempExternal));
    if (maxTemp >= currentPreset.overLimitC) {
      stopProcess(true);
      return;
    }

    // Проверка тайминга фазы
    if (currentPhase < currentPreset.n && runState == RUNNING) {
      uint32_t elapsed = (now - phaseStartMs) / 1000;
      if (elapsed >= currentPreset.phases[currentPhase].seconds) {
        nextPhase();
        if (runState != RUNNING) return;
      }
    }

    // PID управление
    if (currentPhase < currentPreset.n) {
      Phase &phase = currentPreset.phases[currentPhase];
      float dt = float(SAMPLE_MS) / 1000.0f;

      outTop = stepPID(pidTop, phase.targetC, tempTop, dt);
      outBottom = stepPID(pidBottom, phase.targetC, tempBottom, dt);
      outIR = stepPID(pidIR, phase.targetC, tempIR, dt);
    }

    // Логирование
    writeLogLine();
  }

  // Time-proportional управление SSR
  if (now - ssrWindowStart >= SSR_WINDOW) {
    ssrWindowStart = now;
  }
  
  uint32_t timeInWindow = now - ssrWindowStart;
  auto driveSSR = [&](int pin, float percent) {
    uint32_t onTime = (uint32_t)(percent * SSR_WINDOW / 100.0f);
    digitalWrite(pin, timeInWindow < onTime ? HIGH : LOW);
  };

  driveSSR(PIN_SSR_TOP, outTop);
  driveSSR(PIN_SSR_BOTTOM, outBottom);
  driveSSR(PIN_SSR_IR, outIR);
}
