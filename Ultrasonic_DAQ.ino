/*
 * Fanap validation test
 * SRF04 Ultrasonic Sensor - Basic Connection & Data Acquisition
 * Arduino Uno Compatible
 * No external libraries used
 * 
 * Pin Connections:
 * VCC -> 5V
 * GND -> GND  
 * Trig -> Digital Pin 7
 * Echo -> Digital Pin 8
 */

// Pin definitions
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// Constants for distance calculation
const float SOUND_SPEED = 343.0; // Speed of sound in air at 20°C (m/s)
const long TIMEOUT_MICROSECONDS = 30000; // Maximum wait time (30ms for ~5m range)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure pins
  pinMode(TRIG_PIN, OUTPUT); // transmitter
  pinMode(ECHO_PIN, INPUT); // receiver
  
  // Ensure trigger pin starts LOW
  digitalWrite(TRIG_PIN, LOW);
  
  Serial.println("SRF04 Ultrasonic Sensor Initialized");

  // Wait for sensor to stabilize
  delay(500);
}

void loop() {
  // Get distance measurement
  long duration = measureDistance();
  
  if (duration > 0) {
    // Calculate distance in centimeters
    float distanceCm = (duration * SOUND_SPEED * 100.0) / (2.0 * 1000000.0);
    
    // Display results
    Serial.print("Duration: ");
    Serial.print(duration);
    Serial.print(" µs, Distance: ");
    Serial.print(distanceCm, 2);
    Serial.println(" cm");
  } 
  else {
    Serial.println("No echo received - Object out of range or error");
  }
  
  // Wait before next measurement
  delay(500);
}

/**
 * Measures the distance using SRF04/SRF05 ultrasonic sensor
 * Returns: Duration of echo pulse in microseconds (0 if timeout/error)
 */
long measureDistance() {
  // Ensure trigger pin is LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10µs pulse to trigger pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure the duration of echo pulse
  // pulseIn returns 0 if timeout occurs
  long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_MICROSECONDS);
  
  return duration;
}
