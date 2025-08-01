/*
 * SRF04Sensor.h - Ultrasonic Distance Sensor Library
 * Compatible with SRF04 ultrasonic sensors
 * Arduino Uno Compatible 
 * 
 * Features:
 * - Distance measurement in CM/Inch
 * - Object detection with threshold
 * - Filtered readings for mobile robots
 * - Environmental compensation
 * 
 * Author: Amir Abbasi
 * 
 */

#ifndef SRF04SENSOR_H
#define SRF04SENSOR_H

// Distance unit constants
const char UNIT_CM = 'c';
const char UNIT_INCH = 'i';
const char UNIT_DEFAULT = UNIT_CM;

// Default configuration constants
const float DEFAULT_SOUND_SPEED = 343.0;    // m/s at 20°C
const float DEFAULT_MAX_DISTANCE = 400.0;   // cm (4 meters)
const long DEFAULT_TIMEOUT = 30000;         // microseconds
const int FILTER_BUFFER_SIZE = 5;           // readings for median filter
const int FILTER_DELAY_MS = 20;             // delay between filtered readings

class SRF04Sensor {
private:
    // Pin configuration
    int _trigPin;
    int _echoPin;
    
    // Sensor parameters
    float _maxDistance;      // Maximum detection distance (cm)
    long _timeoutMicros;     // Timeout for pulse measurement
    float _soundSpeed;       // Speed of sound (m/s)
    
    // Filter buffer for noise reduction
    long _filterBuffer[FILTER_BUFFER_SIZE];
    int _bufferIndex;
    bool _bufferFull;
    
    // Last valid reading for error recovery
    long _lastValidReading;
    unsigned long _lastReadingTime;
    
    // Private methods
    long measurePulseDuration();
    float convertToDistance(long duration, char unit = UNIT_DEFAULT);
    long applyMedianFilter(long readings[], int size);
    bool isValidReading(long duration);
    void initializeFilter();
    void addToFilterBuffer(long reading);
    void sortArray(long arr[], int size);
    
public:
    // Constructors
    SRF04Sensor(int trigPin, int echoPin);
    SRF04Sensor(int trigPin, int echoPin, float maxDistance);
    
    // Initialization
    void begin();
    
    // Core distance measurement methods
    float getDistance();
    float getDistance(char unit);
    
    // Object detection method
    bool isObjectDetected(float threshold);
    
    // Filtered distance measurement for mobile robots
    float getFilteredDistance();
    float getFilteredDistance(char unit);
    
    // Configuration methods
    void setMaxDistance(float maxDistance);
    void setSoundSpeed(float speedMs);
    void setTimeout(long timeoutMicros);
    
    // Utility methods
    long ping();                    // Raw pulse duration
    float getSoundSpeed();          // Get current sound speed
    float getMaxDistance();         // Get maximum distance setting
    bool isReady();                 // Check if sensor is ready
    
    // Diagnostic methods
    void printDiagnostics();
    bool testSensor();
};

#endif