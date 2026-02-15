/*
 * MQ-9 Gas Sensor with ESP32 - Basic Test Code
 * 
 * This code reads the analog output from the MQ-9 sensor and displays 
 * the raw values and an estimated percentage on the Serial Monitor.
 * 
 * Wiring (for 3-pin sensor modules):
 * VCC -> ESP32 5V (or Vin) - Sensor heater requires 5V for stability
 * GND -> ESP32 GND
 * OUT -> ESP32 GPIO 34 (Analog Input)
 */

// Define the pin connected to the sensor's analog output
const int mq9Pin = 34; 
int sensorValue = 0;

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  
  // Set the sensor pin as an input
  pinMode(mq9Pin, INPUT);
  
  Serial.println("--- MQ-9 Gas Sensor Initialized ---");
  Serial.println("Note: The sensor requires a preheat period to provide stable readings.");
}

void loop() {
  // Read the analog value from the sensor
  // ESP32 ADC resolution is 12-bit (0 - 4095)
  sensorValue = analogRead(mq9Pin);
  
  // Display the raw value
  Serial.print("Raw Sensor Value: ");
  Serial.println(sensorValue);
  
  // Calculate a simple approximate percentage
  float percentage = (sensorValue / 4095.0) * 100.0;
  Serial.print("Approximate Level: ");
  Serial.print(percentage);
  Serial.println("%");
  
  Serial.println("-----------------------");
  
  // Wait for 1 second before the next reading
  delay(1000);
}
