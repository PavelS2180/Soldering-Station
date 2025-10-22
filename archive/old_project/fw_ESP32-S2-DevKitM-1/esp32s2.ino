#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <max6675.h>
#include <PID_v1.h>
#include <SD.h>
#include <SPI.h>

// WiFi настройки
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";
WiFiServer server(80);
WiFiClient client;

// SD-карта настройки
const int chipSelect = 5;

// Пины для датчиков температуры
int thermoCLK = 13; // SCK
int thermoDO = 12;  // SO
int thermoCS_top = 10;
int thermoCS_bottom = 11;
int thermoCS_preheat = 9;
int thermoCS_external = 8;

// Пины для управления нагревателями
int pin_top = 2;
int pin_bottom = 4;
int pin_preheat = 5;

// Пины для управления вентилятором
int pin_fan = 16;

// Пины для кнопок
int button_start = 0;
int button_stop = 15;

// Переменные для хранения значений температуры
double t_top, t_bottom, t_preheat, t_external;

// Переменные для PID-контроллеров
double SetpointTop, InputTop, OutputTop;
double SetpointBottom, InputBottom, OutputBottom;
double SetpointPreheat, InputPreheat, OutputPreheat;

// Настройка PID-контроллеров
PID topPID(&InputTop, &OutputTop, &SetpointTop, 2, 5, 1, DIRECT);
PID bottomPID(&InputBottom, &OutputBottom, &SetpointBottom, 2, 5, 1, DIRECT);
PID preheatPID(&InputPreheat, &OutputPreheat, &SetpointPreheat, 2, 5, 1, DIRECT);

// Инициализация датчиков температуры
MAX6675 thermocouple_top(thermoCLK, thermoCS_top, thermoDO);
MAX6675 thermocouple_bottom(thermoCLK, thermoCS_bottom, thermoDO);
MAX6675 thermocouple_preheat(thermoCLK, thermoCS_preheat, thermoDO);
MAX6675 thermocouple_external(thermoCLK, thermoCS_external, thermoDO);

void setup() {
  // Инициализация серийного порта
  Serial.begin(115200);

  // Настройка пинов
  pinMode(pin_top, OUTPUT);
  pinMode(pin_bottom, OUTPUT);
  pinMode(pin_preheat, OUTPUT);
  pinMode(pin_fan, OUTPUT);
  pinMode(button_start, INPUT_PULLUP);
  pinMode(button_stop, INPUT_PULLUP);

  // Инициализация WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  server.begin();

  // Инициализация SD-карты
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized.");

  // Настройка лимитов для PID-контроллеров
  topPID.SetOutputLimits(0, 255);
  bottomPID.SetOutputLimits(0, 255);
  preheatPID.SetOutputLimits(0, 255);

  // Перевод PID-контроллеров в ручной режим
  topPID.SetMode(MANUAL);
  bottomPID.SetMode(MANUAL);
  preheatPID.SetMode(MANUAL);
}

void loop() {
  // Чтение данных с датчиков температуры
  t_top = thermocouple_top.readCelsius();
  t_bottom = thermocouple_bottom.readCelsius();
  t_preheat = thermocouple_preheat.readCelsius();
  t_external = thermocouple_external.readCelsius();

  // Обработка команд с серийного порта
  if (Serial.available() > 0) {
    processCommand(Serial.readStringUntil('\n'));
  }

  // Обработка команд с WiFi
  client = server.available();
  if (client) {
    if (client.available() > 0) {
      processCommand(client.readStringUntil('\n'));
    }
  }

  // Обновление PID-контроллеров
  updatePID();

  // Управление вентилятором
  if (digitalRead(button_start) == LOW) {
    digitalWrite(pin_fan, HIGH);
  } else if (digitalRead(button_stop) == LOW) {
    digitalWrite(pin_fan, LOW);
  }

  // Логирование данных на SD-карту
  logData();

  // Проверка на аварийные ситуации
  checkForErrors();

  // Отправка данных на компьютер
  sendTemperatureData();
  delay(1000);
}

void processCommand(String command) {
  char cmd = command.charAt(0);
  double setpoint = command.substring(1).toDouble();
  switch (cmd) {
    case 'A':
      SetpointTop = setpoint;
      topPID.SetMode(AUTOMATIC);
      break;
    case 'B':
      SetpointBottom = setpoint;
      bottomPID.SetMode(AUTOMATIC);
      break;
    case 'P':
      SetpointPreheat = setpoint;
      preheatPID.SetMode(AUTOMATIC);
      break;
    case 'T':
      topPID.SetMode(MANUAL);
      digitalWrite(pin_top, LOW);
      break;
    case 'U':
      bottomPID.SetMode(MANUAL);
      digitalWrite(pin_bottom, LOW);
      break;
    case 'H':
      preheatPID.SetMode(MANUAL);
      digitalWrite(pin_preheat, LOW);
      break;
  }
}

void updatePID() {
  if (topPID.GetMode() == AUTOMATIC) {
    InputTop = t_top;
    topPID.Compute();
    analogWrite(pin_top, OutputTop);
  }

  if (bottomPID.GetMode() == AUTOMATIC) {
    InputBottom = t_bottom;
    bottomPID.Compute();
    analogWrite(pin_bottom, OutputBottom);
  }

  if (preheatPID.GetMode() == AUTOMATIC) {
    InputPreheat = t_preheat;
    preheatPID.Compute();
    analogWrite(pin_preheat, OutputPreheat);
  }
}

void sendTemperatureData() {
  String data = "Top: " + String(t_top) + " °C, Bottom: " + String(t_bottom) + " °C, Preheat: " + String(t_preheat) + " °C, External: " + String(t_external) + " °C\n";
  Serial.print(data);
  if (client.connected()) {
    client.print(data);
  }
}

void logData() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print("Top: ");
    dataFile.print(t_top);
    dataFile.print(" °C, Bottom: ");
    dataFile.print(t_bottom);
    dataFile.print(" °C, Preheat: ");
    dataFile.print(t_preheat);
    dataFile.print(" °C, External: ");
    dataFile.print(t_external);
    dataFile.println(" °C");
    dataFile.close();
  } else {
    Serial.println("Error opening datalog.txt");
  }
}

void checkForErrors() {
  if (t_top > 300 || t_bottom > 300 || t_preheat > 300) {
    // Аварийное отключение нагревателей
    topPID.SetMode(MANUAL);
    bottomPID.SetMode(MANUAL);
    preheatPID.SetMode(MANUAL);
    digitalWrite(pin_top, LOW);
    digitalWrite(pin_bottom, LOW);
    digitalWrite(pin_preheat, LOW);
    Serial.println("Error: Overheating detected! Shutting down heaters.");
    if (client.connected()) {
      client.print("Error: Overheating detected! Shutting down heaters.\n");
    }
  }
}
