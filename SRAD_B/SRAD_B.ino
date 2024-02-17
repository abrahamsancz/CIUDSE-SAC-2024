/*
  CIUDSE 
  Spaceport America Cup 2024
  SRAD B
  Lat, Long, No.Sat, | Acce (m/s^2): X, Y, Z, | Gyro (rad/s): X, Y, Z, | Incl (°): X, Y, | Temp (°C), Altit (m), SeaLPress (Pa), RealAltit (m)
*/


#include <SPI.h>
#include <RP2040_SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>


String datos, paquete;
File documento;


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
 

//Reyax
SerialPIO reyax(8, 9);



void setup() 
{

  Serial.begin(115200);

  Serial.println("Inicializando la memoria SD");

  while(!SD.begin(13))
  {
    Serial.println("La inicializacion ha fallado");
  } 

  Serial.println("Inicializacion exitosa");

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


    mpu_read();
    bmp_read();
    

    String paquete = "AT+SEND=5," + String(datos.length()) + "," + datos + "\r\n";
    sendReyax(paquete);

    documento = SD.open("SRAD_B.txt",FILE_WRITE);

    Serial.println(datos);
    documento.print(datos);
    datos = "\0";
    
    documento.close();
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
  delay(100);

  while(reyax.available())
  {
    reyax.write(reyax.read());
  }
}