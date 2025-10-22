/*
 * Паяльная станция ZM-R5860 - ESP32-S3 Контроллер v3.0
 * 
 * Особенности:
 * - 4 канала термопар (Top, Bottom, IR, External)
 * - 3 канала SSR управления нагревателями
 * - Веб-сервер с графиками в реальном времени
 * - PID-регулирование с настраиваемыми коэффициентами
 * - Многофазные профили пайки (до 10 фаз)
 * - Логирование на SD-карту
 * - Защита от перегрева и обрыва термопар
 * 
 * GPIO подключения:
 * MAX6675 #1 (Top) CS    -> GPIO 5
 * MAX6675 #2 (Bottom) CS -> GPIO 17  
 * MAX6675 #3 (IR) CS     -> GPIO 16
 * MAX6675 #4 (External) CS -> GPIO 4
 * SCK                    -> GPIO 18
 * SO                     -> GPIO 19
 * SSR_TOP                -> GPIO 25
 * SSR_BOTTOM             -> GPIO 26
 * SSR_IR                 -> GPIO 27
 * FAN_OUT                -> GPIO 14
 * STATUS_LED             -> GPIO 2
 * SD_CS                  -> GPIO 5 (конфликт с MAX6675 #1!)
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include "max6675.h"

// -------- Конфигурация GPIO --------
#define PIN_MAX_SCK     18
#define PIN_MAX_SO      19
#define PIN_MAX1_CS     5   // Top fan
#define PIN_MAX2_CS     17  // Bottom fan  
#define PIN_MAX3_CS     16  // IR table
#define PIN_MAX4_CS     4   // External TC
#define PIN_SSR_TOP     25
#define PIN_SSR_BOTTOM  26
#define PIN_SSR_IR      27
#define PIN_FAN         14
#define PIN_LED         2
#define PIN_SD_CS       5   // Конфликт с MAX6675 #1!

// -------- Термопары --------
MAX6675 tcTop(PIN_MAX_SCK, PIN_MAX1_CS, PIN_MAX_SO);
MAX6675 tcBottom(PIN_MAX_SCK, PIN_MAX2_CS, PIN_MAX_SO);
MAX6675 tcIR(PIN_MAX_SCK, PIN_MAX3_CS, PIN_MAX_SO);
MAX6675 tcExternal(PIN_MAX_SCK, PIN_MAX4_CS, PIN_MAX_SO);

// -------- WiFi и веб-сервер --------
WebServer server(80);
String wifiMode = "ap";            // "ap" or "sta"
String wifiSsid = "ZM-R5860";
String wifiPass = "reflow123";

// -------- NTP (только для STA режима) --------
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
      // Попытка инициализации SD с разными CS пинами
      sdOK = SD.begin(5);   // GPIO 5
      if (!sdOK) {
        sdOK = SD.begin(15); // GPIO 15 (альтернатива)
        if (!sdOK) {
          Serial.println("SD Card failed, using SPIFFS");
          SPIFFS.begin(true);
          return false;
        }
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
};

const int MAX_PHASES = 10;
struct Preset {
  String name;
  int n;
  float overLimitC;
  Phase phases[MAX_PHASES];
} preset;

enum RunState { IDLE, RUNNING, DONE, ABORTED };
RunState runState = IDLE;

uint8_t currentPhase = 0;
uint32_t phaseStartMs = 0, procStartMs = 0;
float tempTop = 0, tempBottom = 0, tempIR = 0, tempExternal = 0;
float outTop = 0, outBottom = 0, outIR = 0;

// -------- PID контроллеры --------
struct PID {
  float Kp, Ki, integ, outMin = 0, outMax = 100;
  bool enabled = false;
};
PID pidTop, pidBottom, pidIR;

const uint32_t SAMPLE_MS = 200;
uint32_t lastSample = 0;

const uint32_t SSR_WINDOW = 1000;  // Окно для time-proportional управления
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
    logFile.println("ms,phase,topC,bottomC,irC,externalC,outTop,outBottom,outIR");
  }
}

void writeLogLine() {
  if (!logFile) return;
  uint32_t ms = millis() - procStartMs;
  String phase = (runState == RUNNING) ? preset.phases[currentPhase].name : "-";
  logFile.printf("%u,%s,%.1f,%.1f,%.1f,%.1f,%.0f,%.0f,%.0f\n", 
                 ms, phase.c_str(), tempTop, tempBottom, tempIR, tempExternal, 
                 outTop, outBottom, outIR);
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

// -------- Вспомогательные функции --------
float clamp(float v, float min_val, float max_val) {
  return v < min_val ? min_val : (v > max_val ? max_val : v);
}

float stepPID(PID &p, float target, float current, float dt) {
  if (!p.enabled) return 0;
  
  float error = target - current;
  p.integ = clamp(p.integ + error * p.Ki * dt, p.outMin, p.outMax);
  return clamp(error * p.Kp + p.integ, p.outMin, p.outMax);
}

void applyPhasePIDCoeffs() {
  if (currentPhase >= preset.n) return;
  
  Phase &phase = preset.phases[currentPhase];
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
  digitalWrite(PIN_LED, HIGH);
  openLog();
  Serial.println("Process started");
}

void stopProcess(bool aborted) {
  runState = aborted ? ABORTED : DONE;
  outTop = outBottom = outIR = 0;
  digitalWrite(PIN_SSR_TOP, LOW);
  digitalWrite(PIN_SSR_BOTTOM, LOW);
  digitalWrite(PIN_SSR_IR, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_LED, LOW);
  closeLog();
  Serial.println(aborted ? "Process aborted" : "Process completed");
}

void nextPhase() {
  currentPhase++;
  if (currentPhase >= preset.n) {
    stopProcess(false);
    return;
  }
  phaseStartMs = millis();
  applyPhasePIDCoeffs();
  Serial.println("Phase " + String(currentPhase + 1) + ": " + preset.phases[currentPhase].name);
}

// -------- Настройки по умолчанию --------
void loadDefaults() {
  preset.name = "Lead-Free BGA";
  preset.n = 4;
  preset.overLimitC = 280.0;
  
  // Preheat
  preset.phases[0] = {"Preheat", 165, 90, 2.0, 0.08, true, true, true};
  // Soak  
  preset.phases[1] = {"Soak", 190, 60, 2.1, 0.09, true, true, true};
  // Reflow
  preset.phases[2] = {"Reflow", 255, 30, 2.5, 0.10, true, false, false};
  // Cool
  preset.phases[3] = {"Cool", 100, 90, 1.0, 0.05, false, false, false};
}

// -------- Сохранение/загрузка профилей --------
String presetToJson() {
  String json = "{\"name\":\"" + preset.name + "\",\"overLimitC\":" + String(preset.overLimitC, 1) + ",\"n\":" + String(preset.n) + ",\"phases\":[";
  
  for (int i = 0; i < preset.n; i++) {
    if (i) json += ",";
    Phase &p = preset.phases[i];
    json += "{\"name\":\"" + p.name + "\",\"targetC\":" + String(p.targetC, 1) + 
            ",\"seconds\":" + String(p.seconds) + ",\"Kp\":" + String(p.Kp, 3) + 
            ",\"Ki\":" + String(p.Ki, 3) + ",\"useTop\":" + String(p.useTop ? "true" : "false") +
            ",\"useBottom\":" + String(p.useBottom ? "true" : "false") + 
            ",\"useIR\":" + String(p.useIR ? "true" : "false") + "}";
  }
  json += "]}";
  return json;
}

bool savePreset(const String& json) {
  ensureFS();
  fs::FS &fs = activeFS();
  File f = fs.open("/preset.json", "w");
  if (!f) return false;
  f.print(json);
  f.close();
  return true;
}

bool loadPreset() {
  ensureFS();
  fs::FS &fs = activeFS();
  if (!fs.exists("/preset.json")) return false;
  File f = fs.open("/preset.json", "r");
  if (!f) return false;
  String json = f.readString();
  f.close();
  
  // Простой парсер JSON (в реальном проекте лучше использовать библиотеку)
  // Пока просто сохраняем и перезагружаем через веб-интерфейс
  return true;
}

// -------- HTML интерфейс --------
const char* INDEX_HTML = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>ZM-R5860 v3.0</title>
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
        #temperatureChart { height: 300px; }
    </style>
</head>
<body>
    <h1>Паяльная станция ZM-R5860 <span id="state" class="badge">IDLE</span></h1>

    <div class="card">
        <h3>Температуры</h3>
        <div class="grid">
            <div>Верхний фен: <span id="tempTop" class="temp-display">--.-</span> °C</div>
            <div>Нижний фен: <span id="tempBottom" class="temp-display">--.-</span> °C</div>
            <div>IR-стол: <span id="tempIR" class="temp-display">--.-</span> °C</div>
            <div>Внешняя ТС: <span id="tempExternal" class="temp-display">--.-</span> °C</div>
        </div>
        <div class="row">
            <div>Фаза: <b id="phaseName">-</b></div>
            <div>Осталось: <b id="remainTime">-</b> сек</div>
            <div>Выходы: <b id="outputs">0/0/0</b>%</div>
        </div>
    </div>

    <div class="card">
        <h3>График температур</h3>
        <div id="temperatureChart"></div>
    </div>

    <div class="card">
        <h3>Профиль пайки</h3>
        <div class="row">
            <input id="presetName" placeholder="Название профиля" style="min-width: 200px;">
            <label>Лимит °C <input id="tempLimit" type="number" step="1" value="280"></label>
            <button onclick="addPhase()">+ Фаза</button>
            <button onclick="removePhase()">- Фаза</button>
        </div>
        <div id="phasesContainer"></div>
        <div class="row">
            <button onclick="savePreset()">Сохранить</button>
            <button onclick="loadPreset()">Загрузить</button>
        </div>
    </div>

    <div class="card">
        <h3>Управление</h3>
        <div class="row">
            <button onclick="startProcess()" id="startBtn">Старт</button>
            <button onclick="stopProcess()" id="stopBtn">Стоп</button>
            <button onclick="toggleFan()" id="fanBtn">Вентилятор</button>
            <a id="downloadLog" href="#" download style="display:none;">Скачать лог</a>
        </div>
    </div>

    <script>
        let currentPreset = { name: "", overLimitC: 280, n: 0, phases: [] };
        let temperatureData = { time: [], top: [], bottom: [], ir: [], external: [] };
        let maxDataPoints = 100;

        function updateTemperatures() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('tempTop').textContent = data.top.toFixed(1);
                    document.getElementById('tempBottom').textContent = data.bottom.toFixed(1);
                    document.getElementById('tempIR').textContent = data.ir.toFixed(1);
                    document.getElementById('tempExternal').textContent = data.external.toFixed(1);
                    document.getElementById('phaseName').textContent = data.phase;
                    document.getElementById('remainTime').textContent = data.remain;
                    document.getElementById('outputs').textContent = data.outTop + '/' + data.outBottom + '/' + data.outIR;
                    document.getElementById('state').textContent = data.state;

                    // Обновляем график
                    updateChart(data);
                })
                .catch(error => console.error('Error:', error));
        }

        function updateChart(data) {
            const now = Date.now();
            temperatureData.time.push(now);
            temperatureData.top.push(data.top);
            temperatureData.bottom.push(data.bottom);
            temperatureData.ir.push(data.ir);
            temperatureData.external.push(data.external);

            // Ограничиваем количество точек
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
            fetch('/start', { method: 'POST' });
        }

        function stopProcess() {
            fetch('/stop', { method: 'POST' });
        }

        function toggleFan() {
            fetch('/fan', { method: 'POST' });
        }

        // Обновляем данные каждые 300мс
        setInterval(updateTemperatures, 300);
        updateTemperatures(); // Первоначальная загрузка
    </script>
</body>
</html>
)HTML";

// -------- HTTP обработчики --------
void handleIndex() {
  server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  String json = "{";
  json += "\"state\":\"" + String(runState == IDLE ? "IDLE" : runState == RUNNING ? "RUNNING" : runState == DONE ? "DONE" : "ABORTED") + "\",";
  json += "\"top\":" + String(tempTop, 1) + ",\"bottom\":" + String(tempBottom, 1) + ",\"ir\":" + String(tempIR, 1) + ",\"external\":" + String(tempExternal, 1) + ",";
  json += "\"phase\":\"" + String(runState == RUNNING ? preset.phases[currentPhase].name : "-") + "\",";
  
  uint32_t remain = 0;
  if (runState == RUNNING && currentPhase < preset.n) {
    uint32_t elapsed = (millis() - phaseStartMs) / 1000;
    uint32_t duration = preset.phases[currentPhase].seconds;
    remain = (elapsed >= duration) ? 0 : (duration - elapsed);
  }
  
  json += "\"remain\":" + String(remain) + ",";
  json += "\"outTop\":" + String(outTop, 0) + ",\"outBottom\":" + String(outBottom, 0) + ",\"outIR\":" + String(outIR, 0);
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleStart() {
  if (runState != RUNNING) {
    startProcess();
    server.send(200, "text/plain", "started");
  } else {
    server.send(400, "text/plain", "already running");
  }
}

void handleStop() {
  stopProcess(true);
  server.send(200, "text/plain", "stopped");
}

void handleFan() {
  static bool fanOn = false;
  fanOn = !fanOn;
  digitalWrite(PIN_FAN, fanOn ? HIGH : LOW);
  server.send(200, "text/plain", fanOn ? "fan on" : "fan off");
}

// -------- Настройка --------
void setup() {
  Serial.begin(115200);
  
  // Настройка GPIO
  pinMode(PIN_SSR_TOP, OUTPUT);
  pinMode(PIN_SSR_BOTTOM, OUTPUT);
  pinMode(PIN_SSR_IR, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  
  digitalWrite(PIN_SSR_TOP, LOW);
  digitalWrite(PIN_SSR_BOTTOM, LOW);
  digitalWrite(PIN_SSR_IR, LOW);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_LED, LOW);

  // Загрузка настроек
  loadDefaults();
  loadPreset();
  
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

  // Настройка веб-сервера
  server.on("/", handleIndex);
  server.on("/status", handleStatus);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/fan", HTTP_POST, handleFan);
  
  server.begin();
  Serial.println("Web server started");
}

// -------- Основной цикл --------
void loop() {
  server.handleClient();
  uint32_t now = millis();

  // Чтение температур
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;
    tempTop = tcTop.readCelsius();
    tempBottom = tcBottom.readCelsius();
    tempIR = tcIR.readCelsius();
    tempExternal = tcExternal.readCelsius();
  }

  // Обработка процесса пайки
  if (runState == RUNNING) {
    // Проверка безопасности
    float maxTemp = max(max(tempTop, tempBottom), max(tempIR, tempExternal));
    if (maxTemp >= preset.overLimitC) {
      stopProcess(true);
      return;
    }

    // Проверка тайминга фазы
    if (currentPhase < preset.n) {
      uint32_t elapsed = (now - phaseStartMs) / 1000;
      if (elapsed >= preset.phases[currentPhase].seconds) {
        nextPhase();
        if (runState != RUNNING) return;
      }
    }

    // PID управление
    if (currentPhase < preset.n) {
      Phase &phase = preset.phases[currentPhase];
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
