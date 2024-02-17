#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Definir los pines para el GPS
#define RXPin 7
#define TXPin 8
#define GPSBaud 9600

// Inicializar el MPU6050
MPU6050 mpu6050(Wire1);

// Inicializar el BMP280
Adafruit_BMP280 bmp280;

// Inicializar el GPS
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  // Iniciar el MPU6050
  Wire1.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Iniciar el BMP280
  Wire.begin();
  if (!bmp280.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  // Iniciar el GPS
  ss.begin(GPSBaud);
}

void loop() {
  // Leer datos del MPU6050
  mpu6050.update();
  Serial.print("Angle X : ");
  int AnX =mpu6050.getAngleX();
  Serial.print(AnX);
  Serial.print("\tAngle Y : ");
  int AnY = mpu6050.getAngleY();
  Serial.print(AnY);
  Serial.print("\tAngle Z : ");
  int AnZ =mpu6050.getAngleZ();
  Serial.println(AnZ);

  // Leer datos del BMP280
  Serial.print("Temperature = ");
  Serial.print(bmp280.readTemperature());
  Serial.print(" *C\t");
  Serial.print("Pressure = ");
  Serial.print(bmp280.readPressure());
  Serial.print(" Pa\t");
  Serial.print("Approx altitude = ");
  Serial.print(bmp280.readAltitude(1013.25)); // this should be adjusted to your local forcase
  Serial.println(" m");

  // Leer datos del GPS
 while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
    }
 }

  delay(250);
}

