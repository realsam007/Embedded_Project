
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiClient.h>
#include <ArduinoHttpClient.h>
#include <MAX3010x.h>
#include "filters.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <Arduino_LSM6DS3.h>
char ssid[] = "mygalaxy";        // Your Wi-Fi SSID
char password[] = "helliwifi"; // Your Wi-Fi password
char server[] = "192.168.105.1"; 

WiFiClient wifiClient;
HttpClient client = HttpClient(wifiClient, server, 5000); // Port 5000 is the Flask server's port

int status = WL_IDLE_STATUS;


 void sendDataToPi(float bpm, float spo2, float speed, float lat, float lng) {

  // Convert the sensor value to a string formatted for HTTP POST request
  String data = "data=\n BPM VALUE: " + String(bpm)+"\n Oxygen Level: "+String(spo2)
  +"\n Speed: "+String(speed)+"\n Lat: "+String(lat)+"\n Long: "+String(lng);           // Prepare data in "key=value" format

  // Begin HTTP POST request
  client.beginRequest();
  client.post("/data");                                  // POST request to the Raspberry Pi's /data route
  client.sendHeader("Content-Type", "application/x-www-form-urlencoded");
  client.sendHeader("Content-Length", data.length());
  client.sendHeader("Connection", "close");
  client.beginBody();
  client.print(data);                                    // Send data using print() method
  client.endRequest();
  
  Serial.print("Data sent to Raspberry Pi: ");
  Serial.println(bpm);                           // Print sensor value for debugging
  
  delay(5000); // Send data every 5 seconds
}


class MinMaxAvgStatistic {
  float min_;
  float max_;
  float sum_;
  int count_;
public:
  MinMaxAvgStatistic() :
    min_(NAN),
    max_(NAN),
    sum_(0),
    count_(0) {}

  void process(float value) {  
    min_ = isnan(min_) ? value : min(min_, value);
    max_ = isnan(max_) ? value : max(max_, value);
    sum_ += value;
    count_++;
  }

  void reset() {
    min_ = NAN;
    max_ = NAN;
    sum_ = 0;
    count_ = 0;
  }

  float minimum() const {
    return min_;
  }

  float maximum() const {
    return max_;
  }

  float average() const {
    return sum_/count_;
  }
};

// HighPassFilter Class
class HighPassFilter {
  const float kX;
  const float kA0;
  const float kA1;
  const float kB1;
  float last_filter_value_;
  float last_raw_value_;
public:
  HighPassFilter(float samples) :
    kX(exp(-1/samples)),
    kA0((1+kX)/2),
    kA1(-kA0),
    kB1(kX),
    last_filter_value_(NAN),
    last_raw_value_(NAN) {}

  HighPassFilter(float cutoff, float sampling_frequency) :
    HighPassFilter(sampling_frequency/(cutoff*2*PI)) {}

  float process(float value) { 
    if(isnan(last_filter_value_) || isnan(last_raw_value_)) {
      last_filter_value_ = 0.0;
    }
    else {
      last_filter_value_ = 
        kA0 * value 
        + kA1 * last_raw_value_ 
        + kB1 * last_filter_value_;
    }
    
    last_raw_value_ = value;
    return last_filter_value_;
  }

  void reset() {
    last_raw_value_ = NAN;
    last_filter_value_ = NAN;
  }
};

// LowPassFilter Class
class LowPassFilter {
  const float kX;
  const float kA0;
  const float kB1;
  float last_value_;
public:
  LowPassFilter(float samples) :
    kX(exp(-1/samples)),
    kA0(1-kX),
    kB1(kX),
    last_value_(NAN) {}

  LowPassFilter(float cutoff, float sampling_frequency) :
    LowPassFilter(sampling_frequency/(cutoff*2*PI)) {}

  float process(float value) {  
    if(isnan(last_value_)) {
      last_value_ = value;
    }
    else {  
      last_value_ = kA0 * value + kB1 * last_value_;
    }
    return last_value_;
  }

  void reset() {
    last_value_ = NAN;
  }
};

// Differentiator Class
class Differentiator {
  const float kSamplingFrequency;
  float last_value_;
public:
  Differentiator(float sampling_frequency) :
    kSamplingFrequency(sampling_frequency),
    last_value_(NAN) {}

  float process(float value) {  
      float diff = (value-last_value_)*kSamplingFrequency;
      last_value_ = value;
      return diff;
  }

  void reset() {
    last_value_ = NAN;
  }
};

// MovingAverageFilter Class
template<int kBufferSize> class MovingAverageFilter {
  int index_;
  int count_;
  float values_[kBufferSize];
public:
  MovingAverageFilter() :
    index_(0),
    count_(0) {}

  float process(float value) {  
      values_[index_] = value;
      index_ = (index_ + 1) % kBufferSize;
      if(count_ < kBufferSize) {
        count_++;  
      }
      float sum = 0.0;
      for(int i = 0; i < count_; i++) {
          sum += values_[i];
      }
      return sum/count_;
  }

  void reset() {
    index_ = 0;
    count_ = 0;
  }

  int count() const {
    return count_;
  }
};













// Define the threshold for accident detection (in g's)
const float accidentThreshold = 6.0;  // Adjust this value based on testing
TinyGPSPlus gps;
float latitude, longitude;

Adafruit_MPU6050 mpu;

// Define crash detection thresholds
const float ACCELERATION_THRESHOLD = 3.0;  // Threshold in g (1g = 9.81 m/s^2)
const float GYRO_THRESHOLD = 250.0;   


// Sensor (adjust to your sensor type)
MAX30105 sensor;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 5;
const int kSampleThreshold = 5;

// R value to SpO2 calibration factors
const float kSpO2_A = 1.5958422;
const float kSpO2_B = -34.6596622;
const float kSpO2_C = 112.6898759;

// Filter Instances
LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager_bpm;
MovingAverageFilter<kAveragingSamples> averager_r;
MovingAverageFilter<kAveragingSamples> averager_spo2;

// Statistic for pulse oximetry
MinMaxAvgStatistic stat_red;
MinMaxAvgStatistic stat_ir;

// Timestamps
long last_heartbeat = 0;
long finger_timestamp = 0;

// Finger detection and heartbeat flags
bool finger_detected = false;
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

const int hallPin = 2;           // Pin connected to Hall Effect sensor
volatile unsigned long lastTime = 0;  // Last time magnet passed the sensor
volatile unsigned long currentTime = 0; // Current time of magnet detection

// Set your wheel diameter in meters (e.g., 0.7 for a 70 cm diameter wheel)
const float wheelDiameter = 0.7; // Adjust according to your wheel's size
const float wheelCircumference = 3.14159 * wheelDiameter; // Calculate circumference

float speed = 0.0;               // Speed in meters per second
unsigned long stopTimeout = 2000000;  // Timeout in microseconds (2 seconds)

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(hallPin, INPUT_PULLUP); // Use pull-up resistor for hall sensor
  attachInterrupt(digitalPinToInterrupt(hallPin), calculateSpeed, FALLING);


  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network...");
    status = WiFi.begin(ssid, password);
    delay(10000);
  }
  
  Serial.print("Connected to Wi-Fi network with IP address: ");
  Serial.println(WiFi.localIP());




  if (sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor initialized");
  } else {
    Serial.println("Sensor not found");  
    while (1);
  }
}

// Function to reset filters and statistics when finger is removed
void resetFilters() {
  differentiator.reset();
  averager_bpm.reset();
  averager_r.reset();
  averager_spo2.reset();
  low_pass_filter_red.reset();
  low_pass_filter_ir.reset();
  high_pass_filter.reset();
  stat_red.reset();
  stat_ir.reset();
}

// Function to detect finger presence
void detectFinger(float red_value) {
  if (red_value > kFingerThreshold) {
    if (millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  } else {
    resetFilters();
    finger_detected = false;
    finger_timestamp = millis();
  }
}

// Function to calculate SpO2 based on R value
float calculateSpO2(float r_value) {
  return kSpO2_A * r_value * r_value + kSpO2_B * r_value + kSpO2_C;
}

// Function to print results
void printResults(int bpm, float r, float spo2, bool is_average) {

  
  Serial.print("Heart Rate (");
  Serial.print(is_average ? "avg" : "current");
  Serial.print(", bpm): ");
  Serial.println(bpm);



  Serial.print("SpO2 (");
  Serial.print(is_average ? "avg" : "current");
  Serial.print(", %): ");
  Serial.println(spo2);
}

void loop() {
  if ((micros() - lastTime) > stopTimeout) {
    speed = 0.0;
  }

  float x, y, z;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    float accelerationMagnitude = sqrt(x * x + y * y + z * z);
    if (accelerationMagnitude > accidentThreshold) {
      Serial.println("** Accident Detected! **");
    }
  }

  while (Serial1.available()) {
    int data = Serial1.read();
    if (gps.encode(data)) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }
  }

  auto sample = sensor.readSample(1000);
  float red_value = sample.red;
  float ir_value = sample.ir;
  
  detectFinger(red_value);
  if (!finger_detected) return;

  red_value = low_pass_filter_red.process(red_value);
  ir_value = low_pass_filter_ir.process(ir_value);

  stat_red.process(red_value);
  stat_ir.process(ir_value);

  float filtered_value = high_pass_filter.process(red_value);
  float diff = differentiator.process(filtered_value);

  static int bpm = 0;
  static float spo2 = 0.0;

  if (!isnan(diff) && !isnan(last_diff)) {
    if (last_diff > 0 && diff < 0) {
      crossed = true;
      crossed_time = millis();
    }
    if (diff > 0) {
      crossed = false;
    }
    bpm = 60000 / (crossed_time - last_heartbeat);

    if (crossed && diff < kEdgeThreshold) {
      if (last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
        bpm = 60000 / (crossed_time - last_heartbeat);
        float r_value = (stat_red.maximum() - stat_red.minimum()) / stat_red.average();
        float ir_ratio = (stat_ir.maximum() - stat_ir.minimum()) / stat_ir.average();
        float r = r_value / ir_ratio;
        spo2 = calculateSpO2(r);

        if (bpm > 50 && bpm < 250) {
          if (kEnableAveraging) {
            bpm = averager_bpm.process(bpm);
            spo2 = averager_spo2.process(spo2);
          }
        }
        stat_red.reset();
        stat_ir.reset();
      }
      crossed = false;
      last_heartbeat = crossed_time;
    }
  }
  last_diff = diff;

  // Debugging output
  Serial.print("BPM: "); Serial.println(bpm);
  Serial.print("SpO2: "); Serial.println(spo2);
  Serial.print("Speed: "); Serial.println(speed);
  Serial.print("Latitude: "); Serial.println(latitude, 6);
  Serial.print("Longitude: "); Serial.println(longitude, 6);

  sendDataToPi(bpm, spo2, speed, 30.4766, 76.5905);
}


void calculateSpeed() {
  // Interrupt service routine called each time the magnet passes the sensor
  currentTime = micros();  // Record the current time
  unsigned long timeDifference = currentTime - lastTime;

  // Calculate speed if timeDifference is reasonable (ignore if too short)
  if (timeDifference > 10000) { // Avoid noise by setting a minimum time threshold
    speed = wheelCircumference / (timeDifference / 1e6); // Convert µs to seconds
    Serial.print("Speed: ");
    Serial.print(speed * 3.6);
    Serial.println(" km/h");
  }

  lastTime = currentTime; // Update the last time to the current
}















