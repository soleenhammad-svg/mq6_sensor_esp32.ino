/*
 * MQ-6 Gas Sensor with ESP32
 * Detects: LPG, Butane, Propane
 * 
 * Connections:
 * VCC (Sensor) -> 5V or 3.3V (ESP32)
 * GND (Sensor) -> GND (ESP32)
 * AO (Analog Out) -> GPIO 34 (ESP32) - ADC1_CH6
 * DO (Digital Out) -> GPIO 27 (ESP32) - Optional
 */

// Pin Definitions
#define MQ6_ANALOG_PIN 34       // Analog output pin (ADC)
#define MQ6_DIGITAL_PIN 27      // Digital output pin (optional)

// MQ-6 Calibration Constants
#define RL_VALUE 10             // Load resistance in kOhms (typically 10k)
#define RO_CLEAN_AIR_FACTOR 9.83 // RO/RS ratio in clean air
#define CALIBRATION_SAMPLE_TIMES 50
#define CALIBRATION_SAMPLE_INTERVAL 500 // milliseconds

// Gas Detection Thresholds (PPM)
#define LPG_THRESHOLD 300       // LPG threshold in PPM
#define BUTANE_THRESHOLD 300    // Butane threshold in PPM
#define PROPANE_THRESHOLD 300   // Propane threshold in PPM

// Global Variables
float RO = 10;                  // Sensor resistance in clean air
float RS = 0;                   // Sensor resistance under test
float analogValue = 0;          // Raw analog value
float voltage = 0;              // Voltage reading
float PPM = 0;                  // Gas concentration in PPM
int digitalValue = 0;           // Digital output value
bool gasDetected = false;       // Gas detection flag

void setup() {
  // Start Serial Communication
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n");
  Serial.println("=====================================");
  Serial.println("MQ-6 Gas Sensor with ESP32");
  Serial.println("LPG, Butane, Propane Detector");
  Serial.println("=====================================");
  
  // Configure Pins
  pinMode(MQ6_DIGITAL_PIN, INPUT);
  
  Serial.println("\nCalibrating sensor...");
  Serial.println("Please wait 10-15 seconds...");
  
  // Calibrate the sensor
  calibrateSensor();
  
  Serial.println("Calibration complete!");
  Serial.print("RO value: ");
  Serial.println(RO);
  Serial.println("=====================================\n");
  
  delay(1000);
}

void loop() {
  // Read sensor values
  readMQ6Sensor();
  
  // Calculate PPM
  calculatePPM();
  
  // Print sensor data
  printSensorData();
  
  // Delay before next reading
  delay(1000);
}

// Function to calibrate the sensor
void calibrateSensor() {
  float RS_cal = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {
    RS_cal += getRS();
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  
  RS_cal = RS_cal / CALIBRATION_SAMPLE_TIMES;
  RO = RS_cal / RO_CLEAN_AIR_FACTOR;
}

// Function to read MQ-6 sensor
void readMQ6Sensor() {
  // Read analog value
  analogValue = analogRead(MQ6_ANALOG_PIN);
  
  // Convert to voltage (ESP32 ADC is 12-bit, 0-4095)
  voltage = (analogValue / 4095.0) * 3.3;
  
  // Read digital value (optional)
  digitalValue = digitalRead(MQ6_DIGITAL_PIN);
  
  // Calculate RS
  RS = getRS();
}

// Function to calculate RS value
float getRS() {
  int rawADC = analogRead(MQ6_ANALOG_PIN);
  float voltage = (rawADC / 4095.0) * 3.3;
  float RS = (3.3 - voltage) / voltage * RL_VALUE;
  return RS;
}

// Function to calculate PPM
void calculatePPM() {
  float ratio = RS / RO;
  
  // MQ-6 characteristic curve: Y = 10.405 * X^(-1.254)
  // Where Y = PPM, X = RS/RO ratio
  PPM = 10.405 * pow(ratio, -1.254);
  
  // Check if gas is detected
  gasDetected = (PPM > LPG_THRESHOLD);
}

// Function to print sensor data
void printSensorData() {
  Serial.print("Raw ADC: ");
  Serial.print(analogValue);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print("V | RS: ");
  Serial.print(RS, 2);
  Serial.print(" | PPM: ");
  Serial.print(PPM, 2);
  Serial.print(" | Digital: ");
  Serial.print(digitalValue);
  Serial.print(" | Gas Detected: ");
  
  if (gasDetected) {
    Serial.print("YES");
    Serial.print(" (");
    Serial.print(PPM, 1);
    Serial.println(" PPM)");
  } else {
    Serial.println("NO");
  }
}

// Function to get gas type based on PPM
String getGasType() {
  if (PPM > LPG_THRESHOLD) {
    return "LPG/Butane/Propane Detected";
  } else {
    return "No Gas Detected";
  }
}

// Function to set custom threshold
void setGasThreshold(float threshold) {
  if (threshold >= 0) {
    Serial.print("Gas threshold set to: ");
    Serial.print(threshold);
    Serial.println(" PPM");
  } else {
    Serial.println("Error: Threshold must be positive");
  }
}

// Function to get average PPM readings
float getAveragePPM(int samples) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    readMQ6Sensor();
    calculatePPM();
    sum += PPM;
    delay(100);
  }
  return sum / samples;
}

// Function to recalibrate sensor
void recalibrate() {
  Serial.println("Recalibrating sensor...");
  calibrateSensor();
  Serial.print("New RO value: ");
  Serial.println(RO);
}
