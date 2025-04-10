#include <HardwareTimer.h>
#include <TinyGPS++.h>

//USART1 Pin Numbers    //GPS uses this   TX (Transmit): PA9 RX (Receive): PA10
//USART2 Pin Numbers    //UART uses this  TX (Transmit): PA2 RX (Receive): PA3
// Initialize Timer 1
HardwareTimer timer(TIM1);
HardwareSerial SerialGPS(USART1);

// Define pin for onboard LED
#define LED_PIN PC13

// Define GPS baud rate
#define GPS_BAUD 9600
#define GPS_NEWBAUD 9600

// Initialize LED state
bool ledState = false;

// Create TinyGPS++ object
TinyGPSPlus gps;

void setup() {
  // Initialize serial communication for debugging
  Serial2.begin(9600);
  delay(5000);
  Serial2.println("STM32F103C8 PWM Initialized2!");

  // Initialize serial communication for GPS module
  SerialGPS.begin(GPS_BAUD);

#if 0
   SerialGPS.end();
    delay(500); 
#endif    

#if 0
   // UBX-CFG-PRT command to set baud rate to 115200
  byte setBaud[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08,
    0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00
  };

  // Calculate checksum
  byte CK_A = 0, CK_B = 0;
  for (int i = 2; i < sizeof(setBaud) - 2; i++) {
    CK_A += setBaud[i];
    CK_B += CK_A;
  }
  setBaud[sizeof(setBaud)-2] = CK_A;
  setBaud[sizeof(setBaud)-1] = CK_B;
 #endif 

  byte setBaud9600[] = {
  0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x25,
  0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x86, 0x75
};

#if 0
  // Send the command
  SerialGPS.write(setBaud9600, sizeof(setBaud9600));
  delay(500); // Wait for command processing

  // Reopen at new baud rate
  SerialGPS.end();
  SerialGPS.begin(GPS_NEWBAUD);
  Serial.print("Baud rate changed to  ");
  Serial.print(GPS_NEWBAUD);
 #endif 

  // Configure LED pin as output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn off LED initially

  // Configure Timer 1 with a microsecond base
  timer.setPrescaleFactor(SystemCoreClock / 1000000);  // 1 µs per tick

  // Configure PWM mode on PB13 (TIM1 Channel 1)
  timer.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PB13);

  // Start the timer
  timer.resume();

  updatePWM(10, 50.0);
}

void loop() {

 //updatePWM(169, 50.0);
  //digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Read current state & toggle
  //Serial2.print("Looping\r\n");
#if 1
  while (SerialGPS.available() > 0) {
     //Serial2.print("J");
    gps.encode(SerialGPS.read());
  }

  // Check if the GPS data is valid (using isValid() for location)
  if (gps.location.isValid() && gps.speed.isValid()) {
    // Get the speed in km/h
    float speed_kmh = gps.speed.kmph();

    // Get the number of satellites
    int satelliteCount = gps.satellites.value();

    // Get the latitude and longitude
    float latitude = gps.location.lat();
    float longitude = gps.location.lng();

    // Print the current speed, satellites, coordinates, and PWM frequency in one line with proper spacing
    Serial2.print("Speed:        ");
    Serial2.print(speed_kmh, 2);  // Print speed with 2 decimal places
    Serial2.print(" km/h, Satellites: ");
    Serial2.print(satelliteCount);
    Serial2.print(", Latitude:    ");
    Serial2.print(latitude, 6);  // Print latitude with 6 decimal places
    Serial2.print(", Longitude:   ");
    Serial2.print(longitude, 6);  // Print longitude with 6 decimal places

    // Calculate the PWM frequency based on speed
    int pwmFrequency = (int)speed_kmh;
    if (pwmFrequency == 0) pwmFrequency = 1;  // Set to a minimum value if speed is zero

    // Print the frequency with proper spacing
    Serial2.print(", Frequency:   ");
    Serial2.println(pwmFrequency);

    // Update PWM based on speed
    updatePWM(pwmFrequency, 50.0);  // Set 50% duty cycle

    // Toggle the LED every second to indicate activity
    static unsigned long lastToggleTime = 0;
    if (millis() - lastToggleTime >= 200) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState ? LOW : HIGH);
      lastToggleTime = millis();
    }
    delay(50);
  } else {
    // If the GPS location or speed is invalid, do not print or update PWM
    // No print needed
  }
  #endif
  //delay(50);
}

void updatePWM(int frequency, float dutyCycle) {
  if (frequency == 0) return;  // Prevent division by zero

  // Calculate period in microseconds
  unsigned long period = 1000000 / frequency;

  // Adjust pulse widths
  unsigned long positivePulseWidth = (unsigned long)(96.20);  // 96.20 µs positive width
  unsigned long negativePulseWidth = (unsigned long)(3.80);  // 3.80 µs negative width

#if 0
  // Print the frequency and pulse width for debugging
  Serial2.print("Set PWM Frequency: ");
  Serial2.println(frequency);
  Serial2.print("Calculated Period: ");
  Serial2.println(period);
  Serial2.print("Positive Pulse Width: ");
  Serial2.println(positivePulseWidth);
  Serial2.print("Negative Pulse Width: ");
  Serial2.println(negativePulseWidth);
 #endif 

  // Update timer overflow to set new frequency
  timer.setOverflow(period, MICROSEC_FORMAT);

  // Set duty cycle by updating the capture compare register
  // Here, we need to adjust to have a positive and negative pulse within the same period.
  timer.setCaptureCompare(1, positivePulseWidth, MICROSEC_COMPARE_FORMAT);  // Positive width
  timer.setCaptureCompare(2, negativePulseWidth, MICROSEC_COMPARE_FORMAT);  // Negative width
}
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
