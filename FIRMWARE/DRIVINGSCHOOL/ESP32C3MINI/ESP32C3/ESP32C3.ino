#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

bool newData = false;

char receivedMessage[250]; // Buffer to hold received string

// Define LED pins
const int blinked = 8; 
const int LeftLED = 10;    
const int StopLED = 20;    
const int RightLED = 21;   
const int RandomLEDs[] = {10, 20, 21}; 

// Define input switches
const int LeftSwitch = 7;
const int StopSwitch = 8;
const int RightSwitch = 9;

void turnOnRandomLED() {
    int numLEDs = sizeof(RandomLEDs) / sizeof(RandomLEDs[0]);

    // Turn off all LEDs
    for (int i = 0; i < numLEDs; i++) {
        digitalWrite(RandomLEDs[i], LOW);
    }

    // Choose a random LED to turn on
    int randomIndex = random(0, numLEDs);
    digitalWrite(RandomLEDs[randomIndex], HIGH);
    digitalWrite(blinked, HIGH);

    // Keep the chosen LED on for 3 seconds
    delay(5000);
    digitalWrite(RandomLEDs[randomIndex], LOW);
    digitalWrite(blinked, LOW);
    delay(5000);
}

// Callback when data is received via ESP-NOW
void OnDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    if (len < sizeof(receivedMessage)) { 
        memcpy(receivedMessage, incomingData, len);
        receivedMessage[len] = '\0'; 
        newData = true;
    } else {
        Serial.println("Received data is too large for the buffer");
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting setup...");

    // Initialize LED pins
    pinMode(LeftLED, OUTPUT);
    pinMode(StopLED, OUTPUT);
    pinMode(RightLED, OUTPUT);
    pinMode(blinked, OUTPUT);
    
    digitalWrite(blinked, LOW);

    // Initialize input switch pins
    pinMode(LeftSwitch, INPUT_PULLUP);
    pinMode(StopSwitch, INPUT_PULLUP);
    pinMode(RightSwitch, INPUT_PULLUP);

    // Set up WiFi and ESP-NOW
    WiFi.mode(WIFI_MODE_STA);
    Serial.println("WiFi mode set to STA");

    Serial.println("Initializing ESP-NOW...");
    esp_err_t result = esp_now_init();
    if (result != ESP_OK) {
        Serial.print("Error initializing ESP-NOW: ");
        Serial.println(result);
        return;
    }
    Serial.println("ESP-NOW initialized successfully");

    // Register ESP-NOW receive callback
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW receive callback registered");

    delay(2000); // Allow some time for the system to stabilize
}

void loop() {
    // Process received data
    if (newData) {
        Serial.println(receivedMessage);
        if (strcmp(receivedMessage, "SENTH SAFEMAX") == 0) {
            turnOnRandomLED();
        }
        newData = false;
    }

    // Check switch states and turn on corresponding LEDs
    if (digitalRead(LeftSwitch) == LOW) {
        Serial.println("Left switch pressed");
        digitalWrite(LeftLED, HIGH);
    } else {
        digitalWrite(LeftLED, LOW);
    }

    if (digitalRead(StopSwitch) == LOW) {
        Serial.println("Stop switch pressed");
        digitalWrite(StopLED, HIGH);
    } else {
        digitalWrite(StopLED, LOW);
    }

    if (digitalRead(RightSwitch) == LOW) {
        Serial.println("Right switch pressed");
        digitalWrite(RightLED, HIGH);
    } else {
        digitalWrite(RightLED, LOW);
    }

    delay(50);  // Small delay to reduce Serial Monitor flooding
}
