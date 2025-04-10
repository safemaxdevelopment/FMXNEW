#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>

bool newData = false;
char receivedMessage[250]; // Buffer to hold received string

// Define LED pins
const int LeftLED = 18;    
const int StopLED = 19;    
const int RightLED = 21;   
const int RandomLEDs[] = {18, 19, 21}; 

// Define button pins
const int RandomButtonPin = 34;  
const int LeftButtonPin = 32;    
const int StopButtonPin = 25;    
const int RightButtonPin = 27;   

// Button state
bool randomButtonPressed = false;  
bool leftButtonPressed = false;    
bool stopButtonPressed = false;    
bool rightButtonPressed = false;    

// Debounce delay
const unsigned long debounceDelay = 50;  

// Function to turn on a random LED
void turnOnRandomLED() {
    int numLEDs = sizeof(RandomLEDs) / sizeof(RandomLEDs[0]);

    // Turn off all LEDs
    for (int i = 0; i < numLEDs; i++) {
        digitalWrite(RandomLEDs[i], LOW);
    }

    // Choose a random LED to turn on
    int randomIndex = random(0, numLEDs);
    digitalWrite(RandomLEDs[randomIndex], HIGH);

    // Keep the chosen LED on for 3 seconds
    delay(3000);
    digitalWrite(RandomLEDs[randomIndex], LOW);
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

// Function to read button state with debouncing
bool readButton(int pin) {
    // Check if button is LOW (pressed) first, then wait for a HIGH signal
    if (digitalRead(pin) == LOW) {
        return false; // Not pressed
    }

    // Wait for the button to be pressed and debounce
    unsigned long startTime = millis();
    while (digitalRead(pin) == HIGH) {
        if (millis() - startTime > debounceDelay) {
            return true; // Button pressed
        }
    }
    return false; // Button not pressed
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting setup...");

    // Initialize button pins with internal pull-up resistors
    //pinMode(RandomButtonPin, INPUT_PULLUP);
    //pinMode(LeftButtonPin, INPUT_PULLUP);
    //pinMode(StopButtonPin, INPUT_PULLUP);
    //pinMode(RightButtonPin, INPUT_PULLUP);

    pinMode(RandomButtonPin, INPUT);
    pinMode(LeftButtonPin, INPUT);
    pinMode(StopButtonPin, INPUT);
    pinMode(RightButtonPin, INPUT);

    // Initialize LED pins
    pinMode(LeftLED, OUTPUT);
    pinMode(StopLED, OUTPUT);
    pinMode(RightLED, OUTPUT);

    // Ensure all LEDs are initially off
    digitalWrite(LeftLED, LOW);
    digitalWrite(StopLED, LOW);
    digitalWrite(RightLED, LOW);

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
    // Poll Random Button
    if (readButton(RandomButtonPin)) {
        randomButtonPressed = true;
    }

    // Poll Left Button
    if (readButton(LeftButtonPin)) {
        leftButtonPressed = true;
    }

    // Poll Stop Button
    if (readButton(StopButtonPin)) {
        stopButtonPressed = true;
    }

    // Poll Right Button
    if (readButton(RightButtonPin)) {
        rightButtonPressed = true;
    }

    // Handle button presses
    if (randomButtonPressed) {
        Serial.println("BUTTON 1: RANDOM");
        turnOnRandomLED();
        randomButtonPressed = false; // Reset flag
    }

    if (leftButtonPressed) {
        Serial.println("BUTTON 2: LEFT");
        digitalWrite(LeftLED, HIGH);
        delay(3000);
        digitalWrite(LeftLED, LOW);
        leftButtonPressed = false; // Reset flag
    }

    if (stopButtonPressed) {
        Serial.println("BUTTON 3: STOP");
        digitalWrite(StopLED, HIGH);
        delay(3000);
        digitalWrite(StopLED, LOW);
        stopButtonPressed = false; // Reset flag
    }

    if (rightButtonPressed) {
        Serial.println("BUTTON 4: RIGHT");
        digitalWrite(RightLED, HIGH);
        delay(3000);
        digitalWrite(RightLED, LOW);
        rightButtonPressed = false; // Reset flag
    }

    // Process received data
    if (newData) {
        Serial.println(receivedMessage);
        if (strcmp(receivedMessage, "SENTH SAFEMAX") == 0) {
            turnOnRandomLED();
        }
        newData = false;
    }

    delay(50);  // Small delay to reduce Serial Monitor flooding
}
