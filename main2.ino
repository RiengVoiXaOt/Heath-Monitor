#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "Protocentral_MAX30205.h"
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <numeric>
#include <WiFiManager.h>

#define DATABASE_URL "https://healthmonitor-a6b4e-default-rtdb.firebaseio.com/"
#define DATABASE_SECRET "7HKiz6i3KRUnQ9b69ldM83uu4cHVvqCQwLF1OrDP"
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;
String path = "/Health";

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;
MAX30205 tempSensor;

#define BUFFER_SIZE 10
long irBuffer[BUFFER_SIZE];
int irIndex = 0;

const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
float spO2 = 0.0;
float temperatureC = 0.0;

const long MIN_IR_VALUE = 4000;
bool fingerDetected = false;
long lastIRValue = 0;
long lastBeatDetectedTime = 0;
const int beatThreshold = 5000;
const int debounceDelay = 330;

const int MAX_HISTORY = 100;
std::vector<float> hrHistory;

long irMovingAverage = 0;
const int smoothingFactor = 20;
bool isInhaling = false;
bool isExhaling = false;
unsigned long previousMillis = 0;

uint8_t graph[SCREEN_WIDTH];
float spo2Waveform[32] = {
  0.0, 0.2, 0.5, 0.8, 1.0, 1.2, 1.3, 1.5, 1.55, 1.5,1.3, 1.0, 0.8, 0.82, 
  0.84, 0.86, 0.88, 0.92,0.8, 0.7, 0.6, 0.4, 0.3, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

float medianFilter(std::vector<float>& values) {
  std::vector<float> temp = values;
  std::sort(temp.begin(), temp.end());
  return temp[temp.size() / 2];
}

float movingAverage(std::vector<float>& values) {
  if (values.empty()) return 0;
  float sum = std::accumulate(values.begin(), values.end(), 0.0);
  return sum / values.size();
}

bool isOutlier(float current, float previous, float threshold = 11.0) {
  return abs(current - previous) > threshold;
}

float applyIIRFilter(float current, float previous, float alpha = 0.4) {
  return alpha * current + (1 - alpha) * previous;
}

float lastFilteredHR = 0;

float calculateSpO2Simple(long redValue, long irValue) {
  float ratio = (float)redValue / irValue;
  float spO2 = 110.0 - (25.0 * ratio);
  return constrain(spO2, 0.0, 100.0);
}

bool improvedCheckForBeat(long irValue) {
  irBuffer[irIndex] = irValue;
  irIndex = (irIndex + 1) % BUFFER_SIZE;

  int prev = (irIndex + BUFFER_SIZE - 2) % BUFFER_SIZE;
  int curr = (irIndex + BUFFER_SIZE - 1) % BUFFER_SIZE;
  int next = irIndex;

  if (irBuffer[prev] < irBuffer[curr] && irBuffer[curr] > irBuffer[next] && irBuffer[curr] > beatThreshold) {
    if (millis() - lastBeatDetectedTime > debounceDelay) {
      lastBeatDetectedTime = millis();
      return true;
    }
  }
  return false;
}

void drawPlethWave(float amplitude, float bpm, float spo2, float temp) {
  // Cuộn sóng sang trái
  for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
    graph[i] = graph[i + 1];
  }

  static int waveIndex = 0;
  float yOffset = spo2Waveform[waveIndex] * amplitude;
  int baseline = SCREEN_HEIGHT / 2;
  int newY = baseline - yOffset;
  newY = constrain(newY, 0, SCREEN_HEIGHT - 17);
  graph[SCREEN_WIDTH - 1] = newY;
  waveIndex = (waveIndex + 1) % 32;

  // Xóa vùng sóng
  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT - 17, SSD1306_BLACK);

  // Vẽ lại sóng
  for (int i = 0; i < SCREEN_WIDTH - 1; i++) {
    display.drawLine(i, graph[i], i + 1, graph[i + 1], SSD1306_WHITE);
  }

  // Vẽ thông tin bên dưới
  display.fillRect(0, SCREEN_HEIGHT - 17, SCREEN_WIDTH, 17, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, SCREEN_HEIGHT - 16);
  display.print("HR:"); display.print(bpm, 0);
  display.print(" SpO2:"); display.print(spo2, 0);
  display.setCursor(0, SCREEN_HEIGHT - 8);
  display.print("Temp:"); display.print(temp, 1); display.print("C");

  display.display();
}

void setupWiFi() {
  WiFiManager wifiManager;
  // Nếu chưa có WiFi, tạo AP tên "ESP32_ConfigAP" để người dùng nhập thông tin
  if (!wifiManager.autoConnect("ESP32_ConfigAP")) {
    Serial.println("Không kết nối được WiFi, khởi động lại");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Đã kết nối WiFi!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void initFirebase() {
  config.database_url = DATABASE_URL;
  auth.user.email = "";
  auth.user.password = "";
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void setup() {
  Serial.begin(115200);
  setupWiFi();
  initFirebase();
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while (1);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Initializing...");
  display.display();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) while (1);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x12);
  particleSensor.setPulseAmplitudeGreen(0);

  while (!tempSensor.scanAvailableSensors()) delay(5000);
  tempSensor.begin();
}

unsigned long lastUpdate = 0;

void loop() {
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  fingerDetected = (irValue > MIN_IR_VALUE);

  if (fingerDetected) {
    if (improvedCheckForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      float rawHR = 60.0 / (delta / 1000.0);

      if (rawHR < 130 && rawHR > 30) {
        float filteredHR = rawHR;
        if (!hrHistory.empty() && isOutlier(rawHR, hrHistory.back())) {
          filteredHR = hrHistory.back();
        }

        hrHistory.push_back(filteredHR);
        if (hrHistory.size() > MAX_HISTORY) {
          hrHistory.erase(hrHistory.begin());
        }

        float medianHR = medianFilter(hrHistory);
        float averageHR = movingAverage(hrHistory);
        float iirFilteredHR = applyIIRFilter(filteredHR, lastFilteredHR);

        lastFilteredHR = iirFilteredHR;
        beatsPerMinute = iirFilteredHR;

        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
    spO2 = calculateSpO2Simple(redValue, irValue);
    irMovingAverage = (irMovingAverage * (smoothingFactor - 1) + irValue) / smoothingFactor;

    if (millis() - previousMillis > 500) {
        previousMillis = millis();
        temperatureC = tempSensor.getTemperature() + 0.3;

        Serial.print("SpO2: "); Serial.print(spO2);
        Serial.print(" | Raw HR: "); Serial.print(beatsPerMinute);
        Serial.print(" | Avg HR: "); Serial.print(beatAvg);
        Serial.print(" | Temp: "); Serial.println(temperatureC);

        float amplitude = map(spO2, 90, 100, 5, 20);
        amplitude = constrain(amplitude, 5, 20);

        drawPlethWave(amplitude, beatAvg, spO2, temperatureC);

        if (!isnan(beatsPerMinute) && !isnan(spO2) && !isnan(temperatureC)) {
          FirebaseJson json;
          json.set("HR", beatsPerMinute);
          json.set("SpO2", spO2);
          json.set("Temp", temperatureC);

          if (Firebase.setJSON(firebaseData, path, json)) {
            Serial.println("Sent to Firebase");
          } else {
            Serial.println("Firebase Error: " + firebaseData.errorReason());
          }
        } else {
          Serial.println("Warning: NaN detected! Data not sent to Firebase.");
        }
    }
  } else {
    // Reset các giá trị
    beatsPerMinute = 0;
    beatAvg = 0;
    spO2 = 0.0;
    temperatureC = 0.0;
    hrHistory.clear();  // Xóa bộ đệm HR

    // Hiển thị thông báo
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.print("Place your finger");
    display.setCursor(35, 32);
    display.print("on sensor");
    display.display();

    // In Serial trạng thái
    Serial.println("Waiting for finger...");
  }
}
