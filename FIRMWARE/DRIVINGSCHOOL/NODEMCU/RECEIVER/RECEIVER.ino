#include <ESP8266WiFi.h>
#include <espnow.h>

bool newData = false;
char receivedMessage[250]; // Buffer to hold received string

// Define LED pins
const int LeftLED = D1;    
const int StopLED = D2;    
const int RightLED = D3;   
const int RandomLEDs[] = {D1, D2, D3}; 

// Define button pins
const int RandomButtonPin = D5;  
const int LeftButtonPin = D6;    
const int StopButtonPin = D7;    
const int RightButtonPin = D0;    

// Button state
bool randomButtonPressed = false;  
bool leftButtonPressed = false;    
bool stopButtonPressed = false;    

// Debounce delay
const unsigned long debounceDelay = 50;  
unsigned long lastRandomButtonPressTime = 0;
unsigned long lastLeftButtonPressTime = 0;
unsigned long lastStopButtonPressTime = 0;
unsigned long lastRightButtonPressTime = 0;

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
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
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
    pinMode(RandomButtonPin, INPUT_PULLUP);
    pinMode(LeftButtonPin, INPUT_PULLUP);
    pinMode(StopButtonPin, INPUT_PULLUP);
    pinMode(RightButtonPin, INPUT_PULLUP);

    // Initialize LED pins
    pinMode(LeftLED, OUTPUT);
    pinMode(StopLED, OUTPUT);
    pinMode(RightLED, OUTPUT);

    // Ensure all LEDs are initially off
    digitalWrite(LeftLED, LOW);
    digitalWrite(StopLED, LOW);
    digitalWrite(RightLED, LOW);

    // Set up WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi mode set to STA");

    Serial.println("Initializing ESP-NOW...");
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.println("ESP-NOW initialized successfully");

    // Register ESP-NOW receive callback
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW receive callback registered");

    delay(2000); // Allow some time for the system to stabilize
}

void loop() {
    unsigned long currentTime = millis();

    // Poll Random Button
    if (readButton(RandomButtonPin) && (currentTime - lastRandomButtonPressTime > debounceDelay)) {
        lastRandomButtonPressTime = currentTime;
        randomButtonPressed = true;
    }

    // Poll Left Button
    if (readButton(LeftButtonPin) && (currentTime - lastLeftButtonPressTime > debounceDelay)) {
        lastLeftButtonPressTime = currentTime;
        leftButtonPressed = true;
    }

    // Poll Stop Button
    if (readButton(StopButtonPin) && (currentTime - lastStopButtonPressTime > debounceDelay)) {
        lastStopButtonPressTime = currentTime;
        stopButtonPressed = true;
    }

    // Poll Right Button
    if (readButton(RightButtonPin) && (currentTime - lastRightButtonPressTime > debounceDelay)) {
        lastRightButtonPressTime = currentTime;
        Serial.println("BUTTON 4: RIGHT");
        digitalWrite(RightLED, HIGH);
        delay(3000);
        digitalWrite(RightLED, LOW);
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

    // Process received data
    if (newData) {
        Serial.println(receivedMessage);
        if (strcmp(receivedMessage, "SENTH SAFEMAX") == 0) {
            turnOnRandomLED();
        }
        newData = false;

        // Add a small delay to mitigate noise
        delay(50);
    }

    delay(50);  // Small delay to reduce Serial Monitor flooding
}
