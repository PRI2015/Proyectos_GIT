
//Version dataframe datalogger_v01

//Frame rápido
typedef struct dataframe_fast 
{
  byte Head = 0xAA;
  byte ID_disp = 0x01;
  unsigned int N_frame;
    
  int acc_x;
  int acc_y;
  int acc_z;
	
  int giro_x;
  int giro_y;
  int giro_z;
  
  int magn_x;
  int magn_y;
  int magn_z;
  
  long IMU_time;
   
  int Fin = 0xFFFF; // Se guarda en memoria como little endian 
 };

//Frame lento
typedef struct dataframe_slow 
{
  const byte  Head = 0xBB;
  const byte ID_disp = 0x01;
  unsigned int N_frame;
    
  float     temperatura;
  
  int       pres0;
  int       pres1;
  int       pres2;
  int       pres3;
  
  byte      horaRTC;
  byte      minutoRTC;
  byte      segundoRTC;
  
  byte      horaGPS;
  byte      minutoGPS;
  byte      segundoGPS;
  
  float     latitud;
  float     longitud;

  int        estado_bateria;    
  
  const int Fin = 0xFFFF; // Se guarda en memoria como little endian
};
