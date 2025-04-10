#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>

//10:06:1c:b5:ed:40
//cc:db:a7:3d:94:8c
//cc:7b:5c:35:36:0c

//MAC: 08:a6:f7:24:78:c8

//cc:db:a7:3f:70:44

uint8_t receiverMAC[] = {0xCC, 0xDB, 0xA7, 0x3F, 0x70, 0x44};
const int sensorPin = 35; // GPIO35 (ADC1_CH4)

Ticker timer;
bool sensorTriggered = false;

int analogValue = 0;

void onTimer() {

 int sensorState = digitalRead(sensorPin);
 //int sensorState;
 // Serial.println(sensorState);

   //analogValue = analogRead(35);  // Read from GPIO35
  //Serial.println(analogValue);   // Print value

 // if (analogValue == 0)
  //{
      //Serial.println("Sensed");
  //}
  #if 1
  if (sensorState == LOW) { // Assuming active low
    if (!sensorTriggered) { // Trigger only if it was not already triggered
      sensorTriggered = true; // Set the flag

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
 #endif

} 


void setup() {
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP); // Set sensor pin as input
  //pinMode(sensorPin, INPUT);
  delay(1000);  // Short delay to ensure Serial Monitor is ready

  Serial.println("Starting transmitter setup...");

  WiFi.mode(WIFI_MODE_STA);
  Serial.println("WiFi mode set to STA");
  Serial.println("Initializing ESP-NOW...");

  esp_err_t result = esp_now_init();
  if (result != ESP_OK) {
    Serial.print("Error initializing ESP-NOW: ");
    Serial.println(result);
    return;
  } else {
    Serial.println("ESP-NOW initialized successfully");
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  Serial.println("Configuring peer information...");

  result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    Serial.print("Failed to add peer: ");
    Serial.println(result);
    return;
  } else {
    Serial.println("Peer added successfully");
  }

  // Start the ticker to call onTimer every second
  timer.attach(0.001, onTimer); // 1 second
}

void loop() {
  // The loop is empty because all processing is done in the ISR
}
