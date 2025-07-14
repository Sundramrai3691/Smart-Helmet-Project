#include <WiFi.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MAX30105.h>
#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include <TinyGPSPlus.h>

const char* ssid = "HelmetGuardNet";
const char* password = "secure1234";

WiFiServer alertServer(80);

MAX30105 pulseSensor;
Adafruit_MPU6050 imu;
TinyGPSPlus gpsTracker;

HardwareSerial gpsSerial(1);
#define GPS_TX 12
#define GPS_RX 13

SoftwareSerial gsm(17, 16);

#define ALCOHOL_SENSOR 34
#define IR_SENSOR 14
#define BUZZER 25

int alcoholLimit = 500;
int fallLimit = 30;
int minHR = 50;
int minSpO2 = 90;

String contacts[] = {
  "+919188755089", "+919061645091", "+918590607872"
};

int bpm = 0;
int oxygen = 0;
int brainSignal = 0;
bool crash = false;
bool helmetOn = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(IR_SENSOR, INPUT);
  pinMode(ALCOHOL_SENSOR, INPUT);
  pinMode(BUZZER, OUTPUT);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  gsm.begin(9600);

  if (!imu.begin()) {
    Serial.println("MPU error");
    while (1);
  }

  if (!pulseSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("MAX30105 error");
    while (1);
  }
  pulseSensor.setup();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.println(WiFi.localIP());

  alertServer.begin();
}

void loop() {
  WiFiClient link = alertServer.available();

  helmetOn = digitalRead(IR_SENSOR);

  int alcohol = analogRead(ALCOHOL_SENSOR);

  sensors_event_t accel, gyro, tmp;
  imu.getEvent(&accel, &gyro, &tmp);
  float motion = sqrt(accel.acceleration.x * accel.acceleration.x +
                      accel.acceleration.y * accel.acceleration.y +
                      accel.acceleration.z * accel.acceleration.z);
  crash = (motion > fallLimit);

  long irVal = pulseSensor.getIR();
  long redVal = pulseSensor.getRed();
  bpm = map(redVal % 1000, 0, 1000, 60, 100);
  oxygen = map(irVal % 1000, 0, 1000, 90, 100);
  brainSignal = random(40, 95);

  String feed = "LIVE," + String(bpm) + "," + String(oxygen) + "," + String(brainSignal);
  if (link) {
    link.println(feed);
    link.flush();
  }

  if (crash || alcohol > alcoholLimit || bpm < minHR || oxygen < minSpO2) {
    digitalWrite(BUZZER, HIGH);
    triggerAlert(alcohol);
    delay(3000);
  } else {
    digitalWrite(BUZZER, LOW);
  }

  delay(500);
}

void triggerAlert(int alcohol) {
  String statusMsg = (bpm > minHR && oxygen > minSpO2 && alcohol < alcoholLimit) ? "Status: Normal." : "Status: Emergency.";

  String locationURL = "";
  while (gpsSerial.available()) {
    gpsTracker.encode(gpsSerial.read());
    if (gpsTracker.location.isUpdated()) {
      locationURL = "https://maps.google.com/?q=" + String(gpsTracker.location.lat(), 6) + "," + String(gpsTracker.location.lng(), 6);
      break;
    }
  }

  String alertText = statusMsg + "\nHeart Rate: " + String(bpm) +
                     "\nOxygen: " + String(oxygen) +
                     "\nAlcohol: " + String(alcohol) +
                     "\nLocation: " + locationURL;

  for (String num : contacts) {
    gsm.println("AT+CMGF=1");
    delay(100);
    gsm.println("AT+CMGS=\"" + num + "\"");
    delay(100);
    gsm.print(alertText);
    delay(100);
    gsm.write(26);
    delay(3000);
  }
}
