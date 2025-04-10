#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Ticker.h>

//7c:87:ce:88:2e:6e
uint8_t receiverMAC[] = {0x7C, 0x87, 0xCE, 0x88, 0x2E, 0x6E};
const int sensorPin = D1;
bool sensorTriggered = false;
unsigned long lastTriggerTime = 0; // To track the last trigger time
const unsigned long debounceDelay = 200; // 200 ms debounce delay
Ticker timer;

void checkSensor() {
  int sensorState = digitalRead(sensorPin);

  if (sensorState == HIGH) { // Assuming active low
    unsigned long currentTime = millis();
    if (!sensorTriggered && (currentTime - lastTriggerTime > debounceDelay)) {
      sensorTriggered = true; // Set the flag
      lastTriggerTime = currentTime; // Update the last trigger time

      const char* message = "SENTH SAFEMAX";
      if (esp_now_send(receiverMAC, (uint8_t *)message, strlen(message) + 1) == 0) {
        Serial.println("Sent with success");
      } else {
        Serial.println("Error sending the data");
      }
    }
  } else {
    sensorTriggered = false; // Reset the flag if the sensor is not triggered
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP); // Set sensor pin as input
  delay(1000);

  Serial.println("Starting transmitter setup...");

  WiFi.mode(WIFI_STA);
  Serial.println("WiFi mode set to STA");
  Serial.println("Initializing ESP-NOW...");

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW initialized successfully");
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  int result = esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_COMBO, 0, NULL, 0);
  if (result != 0) {
    Serial.print("Failed to add peer: ");
    Serial.println(result);
    return;
  } else {
    Serial.println("Peer added successfully");
  }

  // Start the ticker to call checkSensor every 1ms
  timer.attach(0.001, checkSensor); // 1 ms
}

void loop() {
  // Main loop can do other things or remain empty
}
