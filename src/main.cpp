#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"

#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

// WiFi credentials
constexpr char WIFI_SSID[] = "ACLAB-IOT";
constexpr char WIFI_PASSWORD[] = "12345678";

// ThingsBoard credentials
constexpr char TOKEN[] = "2nm80obclx3l43zw0vc5";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient);
DHT20 dht20;

void checkWiFiTask(void *pvParameters) {
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected. Reconnecting...");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
      }
      Serial.println("Reconnected to WiFi");
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Check every 10 seconds
  }
}

void checkCoreIoTTask(void *pvParameters) {
  while (true) {
    if (!tb.connected()) {
      Serial.println("Connecting to ThingsBoard...");
      if (tb.connect(THINGSBOARD_SERVER, TOKEN)) {
        Serial.println("Connected to ThingsBoard");
      } else {
        Serial.println("Failed to connect to ThingsBoard");
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // Retry in 5 seconds
        continue;
      }
    }
    tb.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);  // Run every 100 ms
  }
}

void telemetryTask(void *pvParameters) {
  while (true) {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (!isnan(temperature) && !isnan(humidity)) {
      Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    } else {
      Serial.println("Failed to read from DHT20 sensor!");
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);  // Send every 10 seconds
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();

  // Create tasks
  xTaskCreate(checkWiFiTask, "Check WiFi", 4096, NULL, 1, NULL);
  xTaskCreate(checkCoreIoTTask, "Check CoreIoT", 4096, NULL, 1, NULL);
  xTaskCreate(telemetryTask, "Telemetry", 4096, NULL, 1, NULL);
}

void loop() {
  delay(1000);
}
