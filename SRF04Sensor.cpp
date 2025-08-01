/*
 * SRF04Sensor.cpp - Implementation of SRF04 Ultrasonic Sensor Library
 * Arduino Uno Compatible 
 */

#include "SRF04Sensor.h"

// Primary Constructor
SRF04Sensor::SRF04Sensor(int trigPin, int echoPin) {
    _trigPin = trigPin;
    _echoPin = echoPin;
    _maxDistance = DEFAULT_MAX_DISTANCE;
    _timeoutMicros = DEFAULT_TIMEOUT;
    _soundSpeed = DEFAULT_SOUND_SPEED;
    _lastValidReading = 0;
    _lastReadingTime = 0;
    initializeFilter();
}

// Overloaded Constructor with max distance
SRF04Sensor::SRF04Sensor(int trigPin, int echoPin, float maxDistance) {
    _trigPin = trigPin;
    _echoPin = echoPin;
    _maxDistance = maxDistance;
    _timeoutMicros = (long)((maxDistance * 2.0 * 1000000.0) / (_soundSpeed * 100.0));
    _soundSpeed = DEFAULT_SOUND_SPEED;
    _lastValidReading = 0;
    _lastReadingTime = 0;
    initializeFilter();
}

void SRF04Sensor::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);
    delay(50); // Allow sensor to stabilize
}

// Main distance measurement method - returns distance in CM by default
float SRF04Sensor::getDistance() {
    return getDistance(UNIT_DEFAULT);
}

// Distance measurement with unit specification
float SRF04Sensor::getDistance(char unit) {
    long duration = measurePulseDuration();
    
    if (isValidReading(duration)) {
        _lastValidReading = duration;
        _lastReadingTime = millis();
        return convertToDistance(duration, unit);
    }
    
    // Return last valid reading if current reading failed
    if (_lastValidReading > 0 && (millis() - _lastReadingTime) < 5000) {
        return convertToDistance(_lastValidReading, unit);
    }
    
    return -1.0; // Error indicator
}

// Object detection within threshold
bool SRF04Sensor::isObjectDetected(float threshold) {
    float distance = getDistance();
    
    if (distance < 0) {
        return false; // Error in reading
    }
    
    return distance <= threshold;
}

// Filtered distance measurement for mobile robots
float SRF04Sensor::getFilteredDistance() {
    return getFilteredDistance(UNIT_DEFAULT);
}

// Filtered distance measurement with unit specification
float SRF04Sensor::getFilteredDistance(char unit) {
    // Take multiple readings over 100ms period
    long readings[FILTER_BUFFER_SIZE];
    int validReadings = 0;
    
    unsigned long startTime = millis();
    
    for (int i = 0; i < FILTER_BUFFER_SIZE && (millis() - startTime) <= 100; i++) {
        long duration = measurePulseDuration();
        
        if (isValidReading(duration)) {
            readings[validReadings] = duration;
            validReadings++;
        }
        
        if (i < FILTER_BUFFER_SIZE - 1) {
            delay(FILTER_DELAY_MS);
        }
    }
    
    if (validReadings == 0) {
        return -1.0; // No valid readings
    }
    
    // Apply median filter
    long filteredDuration = applyMedianFilter(readings, validReadings);
    addToFilterBuffer(filteredDuration);
    
    return convertToDistance(filteredDuration, unit);
}

// Configuration Methods
void SRF04Sensor::setMaxDistance(float maxDistance) {
    _maxDistance = maxDistance;
    _timeoutMicros = (long)((maxDistance * 2.0 * 1000000.0) / (_soundSpeed * 100.0));
}

void SRF04Sensor::setSoundSpeed(float speedMs) {
    _soundSpeed = speedMs;
    _timeoutMicros = (long)((_maxDistance * 2.0 * 1000000.0) / (_soundSpeed * 100.0));
}

void SRF04Sensor::setTimeout(long timeoutMicros) {
    _timeoutMicros = timeoutMicros;
}

// Utility Methods
long SRF04Sensor::ping() {
    return measurePulseDuration();
}

float SRF04Sensor::getSoundSpeed() {
    return _soundSpeed;
}

float SRF04Sensor::getMaxDistance() {
    return _maxDistance;
}

bool SRF04Sensor::isReady() {
    return (_trigPin >= 0 && _echoPin >= 0);
}

// Private Methods Implementation
long SRF04Sensor::measurePulseDuration() {
    // Ensure trigger pin is LOW
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    
    // Send 10µs trigger pulse
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    
    // Measure echo pulse duration with timeout
    long duration = pulseIn(_echoPin, HIGH, _timeoutMicros);
    
    return duration;
}

float SRF04Sensor::convertToDistance(long duration, char unit) {
    if (duration == 0) {
        return -1.0;
    }
    
    // Calculate distance in centimeters
    // Distance = (Duration × Speed of Sound) / 2
    // Duration in microseconds, Speed in m/s, convert to cm
    float distanceCm = (duration * _soundSpeed * 100.0) / (2.0 * 1000000.0);
    
    if (unit == UNIT_INCH || unit == 'I') {
        return distanceCm / 2.54; // Convert to inches
    }
    
    return distanceCm;
}

long SRF04Sensor::applyMedianFilter(long readings[], int size) {
    if (size <= 0) return 0;
    if (size == 1) return readings[0];
    
    // Create a copy for sorting
    long temp[FILTER_BUFFER_SIZE];
    for (int i = 0; i < size; i++) {
        temp[i] = readings[i];
    }
    
    // Sort the array
    sortArray(temp, size);
    
    // Return median value
    return temp[size / 2];
}

bool SRF04Sensor::isValidReading(long duration) {
    if (duration == 0) return false;
    
    float distance = convertToDistance(duration, UNIT_CM);
    return (distance > 2.0 && distance <= _maxDistance);
}

void SRF04Sensor::initializeFilter() {
    _bufferIndex = 0;
    _bufferFull = false;
    
    for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
        _filterBuffer[i] = 0;
    }
}

void SRF04Sensor::addToFilterBuffer(long reading) {
    _filterBuffer[_bufferIndex] = reading;
    _bufferIndex++;
    
    if (_bufferIndex >= FILTER_BUFFER_SIZE) {
        _bufferIndex = 0;
        _bufferFull = true;
    }
}

void SRF04Sensor::sortArray(long arr[], int size) {
    // Simple bubble sort for small arrays
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                long temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// Diagnostic Methods
void SRF04Sensor::printDiagnostics() {
    Serial.println("=== SRF04 Sensor Diagnostics ===");
    Serial.print("Trigger Pin: "); Serial.println(_trigPin);
    Serial.print("Echo Pin: "); Serial.println(_echoPin);
    Serial.print("Max Distance: "); Serial.print(_maxDistance); Serial.println(" cm");
    Serial.print("Sound Speed: "); Serial.print(_soundSpeed); Serial.println(" m/s");
    Serial.print("Timeout: "); Serial.print(_timeoutMicros); Serial.println(" µs");
    Serial.println("================================");
}

bool SRF04Sensor::testSensor() {
    Serial.println("Testing sensor connectivity...");
    
    int successfulReadings = 0;
    const int testReadings = 5;
    
    for (int i = 0; i < testReadings; i++) {
        long duration = measurePulseDuration();
        if (isValidReading(duration)) {
            successfulReadings++;
            Serial.print("Test "); Serial.print(i + 1); 
            Serial.print(": "); Serial.print(convertToDistance(duration));
            Serial.println(" cm - OK");
        } else {
            Serial.print("Test "); Serial.print(i + 1); Serial.println(": FAILED");
        }
        delay(200);
    }
    
    bool testPassed = successfulReadings >= (testReadings / 2);
    Serial.print("Test Result: ");
    Serial.print(successfulReadings); Serial.print("/"); Serial.print(testReadings);
    Serial.println(testPassed ? " - PASSED" : " - FAILED");
    
    return testPassed;
}