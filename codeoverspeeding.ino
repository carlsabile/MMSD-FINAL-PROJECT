#include <LiquidCrystal_I2C.h>  // Library for I2C LCD display
#include <Wire.h>               // Library for I2C communication protocol
#include <TinyGPSPlus.h>        // Library to parse GPS data from NMEA sentences
#include <HardwareSerial.h>     // Hardware serial library for ESP32
#include <WiFi.h>               // WiFi library for ESP32
#include <BlynkSimpleEsp32.h>   // Blynk library for ESP32 devices

// --- Blynk Credentials ---
#define BLYNK_TEMPLATE_ID "TMPL6iRgbMrVV"      // Your Blynk template ID
#define BLYNK_TEMPLATE_NAME "Over Speeding Alert" // Blynk template name (for info)
#define BLYNK_AUTH_TOKEN "HsMkHtJyz7IF2dBNyk_TN_KG4hTetEBu" // Your Blynk auth token

// --- WiFi Credentials ---
char ssid[] = "Wokwi-GUEST";    // WiFi SSID to connect to
char pass[] = "";               // WiFi password (empty if open network)

// --- Pin Definitions ---
#define GPS_RX_PIN 4              // GPIO pin 4 to receive GPS data (RX)
#define GPS_TX_PIN 16             // GPIO pin 16 to transmit to GPS (TX)
#define GREEN_LED_PIN 13          // GPIO pin 13 controls Green LED (safe speed)
#define RED_LED_PIN 12            // GPIO pin 12 controls Red LED (overspeed warning)
#define BUZZER_PIN 14             // GPIO pin 14 controls buzzer (alarm sound)

// --- Objects ---
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Create LCD object at I2C address 0x27, 16x2 size
TinyGPSPlus gps;                      // Create TinyGPSPlus object to parse GPS sentences
HardwareSerial SerialGPS(1);          // Create hardware serial port 1 for GPS communication

// --- Variables ---
float Lat = 0;                       // To store GPS latitude
float Lng = 0;                       // To store GPS longitude
float actualSpeed = 0;               // To store current speed in km/h from GPS
float speedThreshold = 100;          // Speed limit threshold for overspeeding detection (km/h)

unsigned long lastGpsPrint = 0;       // Last timestamp when GPS data was printed to Serial
const unsigned long GPS_PRINT_INTERVAL = 1000; // Interval (1 second) to print GPS info

unsigned long overspeedStartTime = 0; // Time when overspeeding started (millis)
bool overspeeding = false;             // Flag to track if currently overspeeding
bool blynkNotified = false;            // Flag to avoid repeated Blynk notifications
const unsigned long OVERSPEED_DURATION = 5000; // Minimum duration to trigger Blynk alert (5 sec)

void setup() {
  Serial.begin(115200);               // Start serial monitor communication at 115200 baud
  lcd.init();                        // Initialize the LCD
  lcd.backlight();                   // Turn on the LCD backlight

  // Set output pins mode for LEDs and buzzer
  pinMode(GREEN_LED_PIN, OUTPUT);    
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Turn off all indicators initially
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize GPS serial port with 9600 baud rate and custom RX/TX pins
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize I2C for LCD with default SDA=21, SCL=22 pins
  Wire.begin(21, 22);

  delay(3000);                       // Wait 3 seconds for GPS module startup

  Serial.println("Connecting to WiFi and Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);  // Connect to WiFi and Blynk server
}

void loop() {
  Blynk.run();                      // Handle communication with Blynk server

  unsigned long currentMillis = millis();  // Get current system time in ms

  // Read all available GPS characters and feed them to TinyGPS parser
  while (SerialGPS.available()) {
    char c = SerialGPS.read();        // Read one character from GPS serial buffer
    if (gps.encode(c)) {              // Feed to parser; returns true if new valid data
      if (gps.location.isValid()) {  // If location is valid, update latitude and longitude
        Lat = gps.location.lat();
        Lng = gps.location.lng();

        if (gps.speed.isValid()) {   // If speed is valid, update actualSpeed
          actualSpeed = gps.speed.kmph();
        }
      }
    }
  }

  // Print GPS data to Serial Monitor every 1 second
  if (currentMillis - lastGpsPrint >= GPS_PRINT_INTERVAL) {
    if (gps.location.isValid()) {
      Serial.print("Lat: "); Serial.print(Lat, 5);        // Print latitude with 5 decimals
      Serial.print(" , Lng: "); Serial.print(Lng, 5);     // Print longitude with 5 decimals
      Serial.print(" , Speed: "); Serial.print(actualSpeed); // Print speed km/h
      Serial.println(" km/h");
    } else {
      Serial.println("Waiting for GPS fix...");          // No GPS fix yet
    }
    lastGpsPrint = currentMillis;                         // Update last print timestamp
  }

  // Send current speed to Blynk virtual pin V0 for dashboard gauge widget
  Blynk.virtualWrite(V0, actualSpeed);

  // Overspeed detection: if speed exceeds threshold
  if (actualSpeed >= speedThreshold) {
    digitalWrite(GREEN_LED_PIN, LOW);      // Turn off green LED (safe indicator)
    digitalWrite(RED_LED_PIN, HIGH);       // Turn on red LED (warning indicator)
    digitalWrite(BUZZER_PIN, HIGH);        // Turn on buzzer

    if (!overspeeding) {                    // If just started overspeeding
      overspeeding = true;                  // Set overspeeding flag
      overspeedStartTime = currentMillis;  // Record overspeed start time
    } else if ((currentMillis - overspeedStartTime >= OVERSPEED_DURATION) && !blynkNotified) {
      // If overspeeding for more than 5 seconds and no notification sent yet
      Blynk.logEvent("overspeed_alert", "Speed exceeded 100 km/h for over 5 seconds.");
      blynkNotified = true;                 // Set flag to avoid multiple notifications
    }

  } else {
    // If speed is below threshold (normal speed)
    digitalWrite(GREEN_LED_PIN, HIGH);     // Turn on green LED (safe)
    digitalWrite(RED_LED_PIN, LOW);        // Turn off red LED
    digitalWrite(BUZZER_PIN, LOW);         // Turn off buzzer
    overspeeding = false;                   // Clear overspeed flag
    blynkNotified = false;                  // Reset notification flag
  }

  // Display speed on LCD - clear previous content by overwriting spaces
  lcd.setCursor(0, 0);                     
  lcd.print("Speed:           ");           // Label on first line with spaces to clear old digits
  lcd.setCursor(7, 0);
  lcd.print(actualSpeed, 1);                // Print speed with 1 decimal place
  lcd.print("km/h");                        // Print speed unit

  // Display status message on second line
  lcd.setCursor(0, 1);
  if (actualSpeed >= speedThreshold) {
    lcd.print("Status: Overspeeding ");
  } else {
    lcd.print("Status: Normal       ");
  }

  delay(500); // Wait 0.5 seconds before next loop iteration
}
