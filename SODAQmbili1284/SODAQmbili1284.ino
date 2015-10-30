//Programa del prototipo de nodo de adquisición de datos para poder utilizar la tarjeta SODAQ MBILBI1284
//La última modificaciòn la realizó Nelson Gatica el 26 de octubre del 2015 a las 14:29horas

//Biblioteca para utilizar el GPS
#include <TinyGPS++.h>

//Bibliotecas para utilizar la tarjeta microSD
#include <SPI.h>
#include <SdFat.h>

//Bibliotecas para utilizar el sensor de temperatura
#include <OneWire.h> 
#include <DallasTemperature.h>

//Bibliotecas para utilizar la IMU
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>

//Biblioteca para utilizar el RTC de la tarjeta SODAQ mbili
#include <Sodaq_DS3231.h> //también utiliza el wire de mas arriba

//Código para hacer el frame de datos
#include "frame_definition_v01.h"

//Definición de variables y constantes
#define GPSBaud 9600       //velocidad de transmisión del GPS(default)
#define BeeBaud 38400      //velocidad serial de la radio (default)
#define ts_fast 100000     //tiempo de muestreo para sensores rápidos [microsegundos]
#define ts_slow 150        //tiempo de muestreo para sensores lentos
#define t_wakeup_GPS   130 //tiempo para despertar al GPS
#define t_request_temp 140 //tiempo para el request de la temperatura  
#define chipSelect 11      //pin para encender la tarjeta SD (protocolo SPI)
#define Pin_onewire 21     //pin donde se conectará el sensor de temperatura
#define pin_switch_row 22  //pin para prender o apagar sensores de la columna 
#define pin_GND 20         //pin para conectar a GND sensor de temperatura
#define i2c_IMU 0x69       //dirección I2C de la IMU
#define pin_batery A6      //pin analógico para la bateria, 
//#define pin_Sleep_Bee 4    //pin para controlar modo Sleep de la radio
//#define time_wakeup_Bee 2 //tiempo necesario para que la radio despierte, depende del modo Sleep

unsigned long next_fast_sampling; //tiempo para la futura muestra de sensores rápidos
int contador=0;                       //contador
unsigned long cont=0;
//uint8_t buffer_m[6];

//Creación de objetos
TinyGPSPlus gps;
SdFat SD;
File archivo_datos;
MPU9250 accelgyro(0x69); //Dirección I2C 0x69 para la IMU
I2Cdev   I2C_M;
DateTime dt(2015, 12, 19, 12, 17, 0, 5); //Seteo de la hora y fecha del RTC
OneWire ourWire(Pin_onewire); 
DallasTemperature temperatura(&ourWire); 

dataframe_fast Frame_fast;//Estructura para guardar datos
dataframe_slow Frame_slow;

void setup()
{ 
  //Configuración de pines
    pinMode(pin_GND,OUTPUT);
    digitalWrite(pin_GND,0);
    //pinMode(pin_switch_row,OUTPUT);
    //digitalWrite(pin_switch_row,1);
    //pinMode(pin_Sleep_Bee,OUTPUT);
    
  inicio_GPS();
  inicio_SDcard();
  inicio_IMU();
  inicio_tempertura();
  inicio_RTC();  
}

void loop()
{  
 next_fast_sampling=micros()+ts_fast;  //next_fast_sampling es el tiempo para tomar la futura muestra rápida
     Sensores_rapidos();               //Guarda Axyz, Gxyz, Pressión1 y Presión 2
 contador++;
 
 if (contador==t_wakeup_GPS)
 Sleep_GPS(1);
 
 if (contador==t_request_temp)
 temperatura.requestTemperatures();    //Solicitud de temperatura
 
 if (contador==ts_slow)
 Sensores_lentos();                    //Lectura y rescate de sensores lentos
   
      Smart_delay();
}

void Smart_delay()
{
      if ((next_fast_sampling-micros())<ts_fast)
      {  
         while (micros()<next_fast_sampling)
         {
            if(Serial1.available()>0)
            gps.encode(Serial1.read());
         } 
      }
}

void Sensores_lentos()
{ 
    DateTime now = rtc.now(); //get the current date-time

    cont++;
    Frame_slow.N_frame=cont;
      
    Frame_slow.temperatura = temperatura.getTempCByIndex(0);
    Frame_slow.pres0=analogRead(A0);
    Frame_slow.pres1=analogRead(A1);
    Frame_slow.pres2=analogRead(A2);
    Frame_slow.pres3=analogRead(A3);
    
    Frame_slow.horaRTC=now.hour();
    Frame_slow.minutoRTC=now.minute();
    Frame_slow.segundoRTC=now.second();
         
     if (gps.time.isValid())
     {
      Frame_slow.horaGPS    = gps.time.hour();
      Frame_slow.minutoGPS  = gps.time.minute();
      Frame_slow.segundoGPS = gps.time.second();
      Frame_slow.latitud    = gps.location.lat(); 
      Frame_slow.longitud   = gps.location.lng();  
     }
     else
     {
      Frame_slow.horaGPS    = 0xF;
      Frame_slow.minutoGPS  = 0xF;
      Frame_slow.segundoGPS = 0xF;
      Frame_slow.latitud    = 0xFFFF; 
      Frame_slow.longitud   = 0xFFFF;
     }
     Frame_slow.estado_bateria=analogRead(pin_batery);
     Sleep_GPS(0);
     contador=0;

     archivo_datos = SD.open("A11.raw", FILE_WRITE);    

  dataframe_slow *p_Frame_slow = &Frame_slow;
  
      if (archivo_datos) 
      {  archivo_datos.write(p_Frame_slow,32);
         archivo_datos.close();
      }  
      else 
      {
       //Serial.println("Error opening SD");
      }
}

void Sensores_rapidos()
{   
 dataframe_fast *p_Frame_fast = &Frame_fast;       
 cont++;
 Frame_fast.N_frame=cont;
 
 accelgyro.getMotion9(&(Frame_fast.acc_x),&(Frame_fast.acc_y),&(Frame_fast.acc_z),
                      &(Frame_fast.giro_x),&(Frame_fast.giro_y),&(Frame_fast.giro_z),
                      &(Frame_fast.magn_x),&(Frame_fast.magn_y),&(Frame_fast.magn_z));                   
 
 Frame_fast.IMU_time=micros();
  
  archivo_datos = SD.open("A11.raw", FILE_WRITE);    
  
      if (archivo_datos) 
      {  
        archivo_datos.write(p_Frame_fast,28);
        archivo_datos.close();
      }  
      else 
      {
       //Serial.println("Error opening SD");
      }
}

void inicio_GPS()
{
  delay(1000);
  Serial.begin(BeeBaud);//Comienza comunicación serial con Rxo y Txo (radio o monitor serial nunca ambos)
  Serial1.begin(GPSBaud);//Comienza comunicación serial con Rx1 y Tx1 (GPS)

  Serial1.println("$PMTK251,57600*2C"); //Cambio de la velocidad de transmisión del GPS
  delay(1000);
  Serial1.end();                        //Finaliza transmisión Serial 
  Serial1.begin(57600);                 //Inicio y cambio de la velocidad de transmisión/Recepción del Serial1 
  //Serial.println("GPS a " + String(GPSBaud) + " inicializado");
}

void inicio_SDcard()
{
pinMode(chipSelect, OUTPUT);
   if (SD.begin(chipSelect)) 
   {
    //Serial.println("microSD Inicializada.");
   }
   else
   {
    //Serial.println("Error microSD, o no esta presente");
   }  
   delay(1000);
}

void  inicio_IMU()
{
 Wire.begin(i2c_IMU);
 accelgyro.initialize();
    //Serial.println(accelgyro.testConnection() ? "MPU9250 connection succeSerial1ful" : "MPU9250 connection failed"); Serial.println(" ");
    delay(1000); 
}

void inicio_tempertura()
{
  temperatura.begin(); //Comenzar comunicación OneWire
      //Serial.println("Sensor de Temperatura inicializado");
      delay(1000);
}

void inicio_RTC()
{  
 rtc.begin();
 rtc.setDateTime(dt); //Adjuste del reloj 
    //Serial.println("RTC inicializado");
    delay(1000);
}

void Sleep_GPS(int estado)
{
 if (estado==0)
 Serial1.println("$PMTK225,1,1000,16000,0,0*1C");//modo SLEEP "Periodc backup mode" revisado OK
  
   if (estado==1)
   Serial1.println("$PMTK225,0*2B");//modo SLEEP "Periodc backup mode"
}

//void envio_radio(String data)
//{ 
  //digitalWrite(pin_Sleep_Bee,0);
  //delay(time_wakeup_Bee);
  //Serial.println(data);
  //digitalWrite(pin_Sleep_Bee,1);
//}

