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
  lastLogPath = getTimestampFilename();
  logFile = fs.open(lastLogPath, FILE_WRITE);
  if (logFile) {
    logFile.println("ms,phase,topC,bottomC,irC,externalC,outTop,outBottom,outIR,autotune");
  }
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
  Serial.println("Process started - fans set for phase: " + phase.name);
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

  // Загрузка профилей
  loadProfiles();
  
  // Инициализация файловой системы
  SPIFFS.begin(true);
  ensureFS();

  // Настройка WiFi
  if (wifiMode == "sta") {
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    uint32_t timeout = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - timeout < 10000) {
      delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
      setupTime();
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
    } else {
      WiFi.mode(WIFI_AP);
      WiFi.softAP("ZM-R5860", "reflow123");
      Serial.println("AP mode: 192.168.4.1");
    }
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ZM-R5860", "reflow123");
    Serial.println("AP mode: 192.168.4.1");
  }

  // Настройка OTA
  setupOTA();

  // Настройка веб-сервера
  server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });
  server.on("/api/status", handleApiStatus);
  server.on("/api/start", HTTP_POST, handleApiStart);
  server.on("/api/stop", HTTP_POST, handleApiStop);
  server.on("/api/fan", HTTP_POST, handleApiFan);
  server.on("/api/autotune", HTTP_POST, handleApiAutotune);
  server.on("/api/profiles", HTTP_GET, handleApiProfiles);
  server.on("/api/profiles", HTTP_POST, handleApiProfiles);
  
  server.begin();
  Serial.println("Web server started");
  beepSequence(2);
}

// -------- Основной цикл --------
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  updateButtons();
  
  uint32_t now = millis();

  // Чтение температур
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    tempTop = tcTop.readCelsius();
    tempBottom = tcBottom.readCelsius();
    tempIR = tcIR.readCelsius();
    tempExternal = tcExternal.readCelsius();
    
    // Проверка термопар
    checkThermocouple(0, tempTop);
    checkThermocouple(1, tempBottom);
    checkThermocouple(2, tempIR);
    checkThermocouple(3, tempExternal);
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
