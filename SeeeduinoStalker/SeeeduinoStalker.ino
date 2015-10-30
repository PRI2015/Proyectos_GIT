//Programa del prototipo de nodo de adquisición de datos para poder utilizar la tarjeta Seeeduino Stalker v3.0
//Tiene todos los sensores menos el GPS, y respecto de versiones anteriores añade la lectura de la bateria
//La última modificación la realizó Nelson Gatica el lunes 26 de octubre del 2015 a las 17:01horas

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

//Biblioteca para utilizar el RTC de la tarjeta Seeeduino Stalker v3.0
#include <DS1337.h> //también utiliza el wire de mas arriba

//Código para hacer el frame de datos
#include "frame_definition.h"

//Definición de variables y constantes
#define BeeBaud 38400      //velocidad serial de la radio (default)
#define ts_fast 100000     //tiempo de muestreo para sensores rápidos [microsegundos]
#define ts_slow 150        //tiempo de muestreo para sensores lentos
#define t_request_temp 140 //tiempo para el request de la temperatura  
#define chipSelect 10      //pin para encender la tarjeta SD (protocolo SPI)
#define Pin_onewire 6      //pin donde se conectará el sensor de temperatura
#define i2c_IMU 0x69       //dirección I2C de la IMU
#define pin_batery A6      //pin analógico para la bateria, 

unsigned long next_fast_sampling; //tiempo para la futura muestra de sensores rápidos
int contador=0;                       //contador
unsigned long cont=0;

//Creación de objetos
SdFat SD;
File archivo_datos;
MPU9250 accelgyro(0x69); //Dirección I2C 0x69 para la IMU
I2Cdev   I2C_M;
DS1337 RTC; //Create the DS3231 object
DateTime dt(2015, 10, 26, 13, 30, 0, 1); //Seteo de la hora y fecha del RTC
OneWire ourWire(Pin_onewire); 
DallasTemperature temperatura(&ourWire); 

dataframe_fast Frame_fast;//Estructura para guardar datos
dataframe_slow Frame_slow;

void setup()
{     
  //Serial.begin(BeeBaud); //Inicio comunicación serial
  
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
            
         } 
      }
}

void Sensores_lentos()
{ 
    DateTime now = RTC.now();
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
    Frame_slow.estado_bateria=analogRead(pin_batery);
    //Serial.println(Frame_slow.estado_bateria);
    
     contador=0;

     archivo_datos = SD.open("A13.raw", FILE_WRITE);    

  dataframe_slow *p_Frame_slow = &Frame_slow;
  
      if (archivo_datos) 
      {  archivo_datos.write(p_Frame_slow,23);
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
  
  archivo_datos = SD.open("A13.raw", FILE_WRITE);    
  
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
   // Serial.println(accelgyro.testConnection() ? "MPU9250 connection succeSerial1ful" : "MPU9250 connection failed"); Serial.println(" ");
    delay(1000); 
}

void inicio_tempertura()
{
   temperatura.begin(); //Comenzar comunicación OneWire
   //   Serial.println("Sensor de Temperatura inicializado");
      delay(1000);
}

void inicio_RTC()
{  
    RTC.begin();
    RTC.adjust(dt);
 //   Serial.println("RTC inicializado");
    delay(1000);
}
