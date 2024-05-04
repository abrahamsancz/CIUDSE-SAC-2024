/*
  CIUDSE 
  Spaceport America Cup 2024
  SRAD A
  Lat, Long, No.Sat, | Acce (m/s^2): X, Y, Z, | Gyro (rad/s): X, Y, Z, | Incl (°): X, Y, | Temp (°C), Altit (m), SeaLPress (Pa), RealAltit (m)
*/


#include <SPI.h>
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
#define gpsSerial Serial1

const int tamano_sentencia = 120;
char sentencia[tamano_sentencia];
double a = 0, b = 0, c = 0, segundos = 0, totalat = 0, totalong = 0 ;
int grados = 0, minutos = 0;
 

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


// E-matches
int ematchMain = 27;
int ematchDrogue = 28;


// Buzzer
int buzzer = 26;
int frequency = 1000;
int beeps = 3;



void setup() 
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

  // GY-87
  mpu.begin();
  mpu.setI2CBypass(true);
  bmp.begin();
  mpu_temp = mpu.getTemperatureSensor();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();
  altura_inicial = bmp.readAltitude();

  // GPS
  gpsSerial.setRX(1);
  gpsSerial.setTX(0);
  gpsSerial.begin(9600);
}


void loop() 
{
  static int i = 0;
  if (gpsSerial.available()) 
  {
    char dato = gpsSerial.read();
    if (dato != '\n' && i < tamano_sentencia) 
    {
      sentencia[i] = dato;
      i++;
    } 
    else 
    {
      sentencia[i] = '\0';
      i = 0;
      //asyBuzzer.update();
      mostrarDatos();
    }
  } 
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


void mostrarDatos() 
{
  char campo[20], lat[10], lon[10];
  obtener_campo(campo, 0);
  dtostrf(totalat, 6, 6, lat);
  dtostrf(totalong, 6, 6, lon);

  if (strcmp(campo, "$GPGGA") == 0) 
  {
    // Lat
    obtener_campo(campo, 2);
    a = atof(campo) / 100;
    grados = a;
    b = a - grados;
    minutos = (b / 60);
    c = ((b * 60) - minutos) * 100;
    segundos = c / 3600;
    totalat = grados + minutos + segundos;
    datos += lat;
    datos += ";";

    // Long
    obtener_campo(campo, 4);
    a = atof(campo) / 100;
    grados = a;
    b = a - grados;
    minutos = (b / 60);
    c = ((b * 60) - minutos) * 100;
    segundos = c / 3600;
    totalong = -(grados + minutos + segundos);
    datos += lon;
    datos += ";";

    // No. Sat
    obtener_campo(campo, 8);
    datos += campo;


    // GY-87
    mpu_read();
    bmp_read();

    // E-matches
    /*if(altura_real > 1)
    {
      //delay(3000);
      Serial.print("Se activo el E-match Drogue");
      digitalWrite(28, 1);
    }*/

    /* if(altura_real > 1)
    {
      //delay(3000);
      Serial.print("Se activo el E-match Main");
      digitalWrite(ematchMain, 1);
    } */

    // Airbrakes
    if(altura_real > 0.9 && altura_maxima < 1 && bandera == false)
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
    }


    if(altura_real < altura_maxima && altura_real > 0.5 && bandera == true)
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
    }

    altura_maxima = altura_real;
    datos += altura_maxima;


    String paquete = "AT+SEND=0," + String(datos.length()) + "," + datos + "\r\n";
    sendReyax(paquete);

    Serial.println(datos);
    datos = "\0";
  }
}


void obtener_campo(char* buffer, int indice) 
{
  int sentencia_pos = 0;
  int posicion_campo = 0;
  int contador_comas = 0;

  while (sentencia_pos < tamano_sentencia) 
  {
    if (sentencia[sentencia_pos] == ',') 
    {
      contador_comas++;
      sentencia_pos++;
    }

    if (contador_comas == indice) 
    {
      buffer[posicion_campo] = sentencia[sentencia_pos];
      posicion_campo++;
    }

    sentencia_pos++;
  }

  buffer[posicion_campo] = '\0';
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