/*
 * Sound Sensor KY-038 with ESP32
 * 
 * Connections:
 * VCC (Sensor) -> 3.3V (ESP32)
 * GND (Sensor) -> GND (ESP32)
 * DO (Digital Out) -> GPIO 27 (ESP32)
 * AO (Analog Out) -> GPIO 34 (ESP32) - ADC1_CH6
 */

// Pin Definitions
#define SOUND_DIGITAL_PIN 27    // Digital Output
#define SOUND_ANALOG_PIN 34     // Analog Output (ADC)

// Global Variables
int digitalValue = 0;           // Digital output value
int analogValue = 0;            // Analog output value
int soundThreshold = 2000;      // Sound threshold (0-4095)
bool soundDetected = false;     // Sound detection flag

void setup() {
  // Start Serial Communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("=====================================");
  Serial.println("Sound Sensor KY-038 with ESP32");
  Serial.println("=====================================");
  
  // Configure Pins
  pinMode(SOUND_DIGITAL_PIN, INPUT);      // Digital output as input
  // No need for pinMode for analog input
  
  Serial.println("\nSystem Ready!");
  Serial.println("=====================================\n");
}

void loop() {
  // Read sensor values
  readSoundSensor();
  
  // Print sensor data
  printSensorData();
  
  // Small delay
  delay(500);
}

// Function to read sound sensor
void readSoundSensor() {
  // Read digital output
  digitalValue = digitalRead(SOUND_DIGITAL_PIN);
  
  // Read analog output
  analogValue = analogRead(SOUND_ANALOG_PIN);
  
  // Check if threshold is exceeded
  soundDetected = (analogValue > soundThreshold);
}

// Function to print sensor data
void printSensorData() {
  Serial.print("Analog Value: ");
  Serial.print(analogValue);
  Serial.print(" | Digital Value: ");
  Serial.print(digitalValue);
  Serial.print(" | Sound Detected: ");
  
  if (soundDetected) {
    Serial.println("YES");
  } else {
    Serial.println("NO");
  }
}

// Optional Function: Set sound threshold
void setSoundThreshold(int threshold) {
  if (threshold >= 0 && threshold <= 4095) {
    soundThreshold = threshold;
    Serial.print("Sound threshold set to: ");
    Serial.println(soundThreshold);
  } else {
    Serial.println("Error: Value must be between 0 and 4095");
  }
}

// Optional Function: Get average analog readings
int getAverageAnalogValue(int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(SOUND_ANALOG_PIN);
    delay(10);
  }
  return sum / samples;
}
