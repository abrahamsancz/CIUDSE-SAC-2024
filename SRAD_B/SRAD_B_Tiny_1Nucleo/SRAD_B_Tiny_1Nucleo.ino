/*
  CIUDSE 
  Spaceport America Cup 2024
  SRAD B prueba
  Lat, Long, No.Sat, | Acce (m/s^2): X, Y, Z, | Gyro (rad/s): X, Y, Z, | Incl (°): X, Y, | Temp (°C), Altit (m), SeaLPress (Pa), RealAltit (m)
*/


#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>


// GY-87
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

float altura_real = 0, altura_inicial = 0;


// GPS
TinyGPS gps;
SoftwareSerial ss(1, 0);
float flat, flon;


// Reyax
String datos, paquete;
SerialPIO reyax(8, 9);


// SD
File documento;

const int _MISO = 12;
const int _MOSI = 15;
const int _CS = 13;
const int _SCK = 14; 


// Buzzer
int buzzer = 26;
int frequency = 500;
float bandbuzz = false;



void setup() 
{
  Serial.begin(115200);

  // Buzzer
  pinMode(buzzer, OUTPUT);

  // SD
  SD.begin(_CS, SPI1);
  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);

  // Reyax
  reyax.begin(115200);
  sendReyax("AT+MODE=0\r\n");

  // GY-87
  mpu.begin();
  mpu.setI2CBypass(true);
  bmp.begin();
  mpu_temp = mpu.getTemperatureSensor();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();
  altura_inicial = bmp.readAltitude();

  // GPS
  ss.begin(9600);
}

void loop() 
{
  gpss();
  mpu_read();
  bmp_read();
  Buzzzer();

  String paquete = "AT+SEND=0," + String(datos.length()) + "," + datos + "\r\n";
  sendReyax(paquete);

  documento = SD.open("SRAD_B.txt", FILE_WRITE);

  Serial.println(datos);
  documento.print(datos);
  documento.print("\n");
  datos = "\0";

  documento.close(); 
}


void mpu_read() 
{
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  float accel_ang_x = atan(accel.acceleration.x / sqrt(pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(accel.acceleration.y / sqrt(pow(accel.acceleration.x, 2) + pow(accel.acceleration.z, 2))) * (180.0 / 3.14);

  // acceleration is measured in m/s^2
  datos += ";";
  datos += accel.acceleration.x;
  datos += ";";
  datos += accel.acceleration.y;
  datos += ";";
  datos += accel.acceleration.z;
  datos += ";";

  // rotation is measured in grados/s
  datos += gyro.gyro.x*57.296;
  datos += ";";
  datos += gyro.gyro.y*57.296;
  datos += ";";
  datos += gyro.gyro.z*57.296;
  datos += ";";

  // la inclinacion esta dada en grados
  datos += accel_ang_x;
  datos += ";";
  datos += accel_ang_y;
  datos += ";";
}


void bmp_read() 
{
  datos += bmp.readTemperature();
  datos += ";";
  datos += bmp.readAltitude(101592);
  datos += ";";
  datos += bmp.readSealevelPressure();
  datos += ";";
  altura_real = bmp.readAltitude() - altura_inicial;
  datos += altura_real;
}


void gpss()
{
  for (unsigned long start = millis(); millis() - start < 100;) // delay de datos
  {
    while (ss.available())
    {
      char c = ss.read();
      gps.encode(c);
    } 
  }

  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  datos += ",";
  datos += String(flat, 6);
  datos += ";";
  datos += String(flon, 6);
  datos += ";";
  datos += gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
}


void Buzzzer()
{
  if(flat < 100 && flon < 1 && bandbuzz == false)
  {
    for (int i = 0; i < 3; i++) 
    {
      tone(buzzer, frequency); 
      delay(50    -0);             
      noTone(buzzer);      
      delay(500);             
    }

    bandbuzz = true;
  }
  else if(bandbuzz == false)
  {
    tone(buzzer, frequency);
    delay(100);
  }
}


void sendReyax(String paquete)
{
  reyax.print(paquete);
  delay(150); // 100 ms para la SRAD A y 150 ms para la SRAD B

  while(reyax.available())
  {
    reyax.write(reyax.read());
  }
}