/*
  CIUDSE 
  Spaceport America Cup 2024
  SRAD A
  Lat, Long, No.Sat, | Acce (m/s^2): X, Y, Z, | Gyro (rad/s): X, Y, Z, | Incl (°): X, Y, | Temp (°C), Altit (m), SeaLPress (Pa), RealAltit (m), | States: Drogue Ematch, Main Ematch, Airbrakes
*/


#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include "EasyBuzzer.h"
#include "RP2040_ISR_Servo.h"


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


// Reyax
String datos, paquete;
SerialPIO reyax(8, 9);


// Airbrakes
#if ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)
  #if !defined(RP2040_ISR_SERVO_USING_MBED)    
    #define RP2040_ISR_SERVO_USING_MBED     false
  #endif  

#elif ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)
      
  #if !defined(RP2040_ISR_SERVO_USING_MBED)    
    #define RP2040_ISR_SERVO_USING_MBED     true
  #endif  
  
#endif

#define ISR_SERVO_DEBUG             4

#define MIN_MICROS        800
#define MAX_MICROS        2450

#define SERVO_PIN_1       29

typedef struct
{
  int     servoIndex;
  uint8_t servoPin;
} ISR_servo_t;


#define NUM_SERVOS            1

ISR_servo_t ISR_servo[NUM_SERVOS] =
{
  { -1, SERVO_PIN_1 }
};

bool bandera = false;
int altura_maxima = -10;
String airbrakeState = "0";


// E-matches
int ematchMain = 27;
int ematchDrogue = 28;
String drogueState = "0", mainState = "0";


// Buzzer
int buzzer = 26;
int frequency = 1000;
int beeps = 3;


/* Core 0 */

void setup() 
{
  Serial.begin(115200);

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

  String paquete = "AT+SEND=0," + String(datos.length()) + "," + datos + "\r\n";
  sendReyax(paquete);

  Serial.println(datos);
  datos = "\0";
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

  //Ematches
  datos += ";";
  datos += drogueState;
  datos += ";";
  datos += mainState;
  datos += ";";

  // Airbrakes
  datos += airbrakeState;
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

  float flat, flon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);
  datos += ",";
  datos += String(flat, 6);
  datos += ";";
  datos += String(flon, 6);
  datos += ";";
  datos += gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();
}


/* Core 1 */

void setup1() 
{
  Serial.begin(115200);

  // Airbrakes
  for (int index = 0; index < NUM_SERVOS; index++)
  {
    pinMode(ISR_servo[index].servoPin, OUTPUT);
    digitalWrite(ISR_servo[index].servoPin, LOW);
  }

  for (int index = 0; index < NUM_SERVOS; index++)
  {
    ISR_servo[index].servoIndex = RP2040_ISR_Servos.setupServo(ISR_servo[index].servoPin, MIN_MICROS, MAX_MICROS);
  }

  // E-matches
  pinMode(ematchMain, OUTPUT);
  pinMode(ematchDrogue, OUTPUT);

  // Buzzer
  EasyBuzzer.setPin(buzzer);
  EasyBuzzer.beep(frequency, beeps);

  // Reyax
  reyax.begin(115200);
  sendReyax("AT+MODE=0\r\n");
}

void loop1() 
{
  Ematches();
  Airbrakes();
}

void Ematches()
{
  digitalWrite(ematchDrogue, 0);
  digitalWrite(ematchMain, 0);

  if(altura_real < 0)
  {
    drogueState = "1";

    digitalWrite(ematchDrogue, 1);
    delay(100);
    digitalWrite(ematchDrogue, 0);  
  } 

  if(altura_real < 1)
  { 
    mainState = "1";

    digitalWrite(ematchMain, 1);
    delay(100);
    digitalWrite(ematchMain, 0);
  } 
}

void Airbrakes()
{
  if(altura_real > 0.9 && altura_maxima > 90 && bandera == false)
  {
    int position; 

    for (position = 0; position <= 105; position += 105)
    {
      for (int index = 0; index < NUM_SERVOS; index++)
      {
        RP2040_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position);
        bandera = true;
      }
    }

    airbrakeState = "1";
  }

  if(altura_real < altura_maxima && altura_real > 90 && bandera == true)
  {
    int position; 

    for (position = 105; position >= 0; position -= 105)
    {
      for (int index = 0; index < NUM_SERVOS; index++)
      {
        RP2040_ISR_Servos.setPosition(ISR_servo[index].servoIndex, position);
        bandera = false;
      }
    }

    airbrakeState = "1";
  } 

  altura_maxima = altura_real;
  datos += altura_maxima;
}

void sendReyax(String paquete)
{
  reyax.print(paquete);
  delay(50);

  while(reyax.available())
  {
    reyax.write(reyax.read());
  }
}