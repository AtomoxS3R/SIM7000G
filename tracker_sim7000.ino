
#define SerialAT  Serial1
#define SerialMon Serial
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024


const char apn[]  = "fi.omv.es";
const char gprsUser[] = "";
const char gprsPass[] = "";

#define GSM_PIN "2336"
#define SMS_TARGET  "+34756610045"

#include <TinyGPS.h>
#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>



#define DUMP_AT_COMMANDS
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define TIME_TO_SLEEP       60
#define UART_BAUD           115200
#define PIN_DTR             25
#define PIN_TX              27
#define PIN_RX              26
#define PWR_PIN             4

#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13
#define LED_PIN             12

const String PHONE = "+34756610045";

int gnss_run_status = 0;
int fix_status = 0;
int year = 0;
int month = 0;
int day = 0;
int hour = 0;
int minutes = 0;
float secondWithSS = 0;
float lat = 0;
float lon = 0;
float msl_alt = 0;
float speed_over_ground = 0;
float course_over_ground = 0;
bool reserved1 = 0;
int fix_mode = 0;
int   hdop = 0;
int   pdop = 0;
int vdop = 0;
bool reserved2       = 0;
int gnss_satellites_used       = 0;
int gps_satellites_used       = 0;
int glonass_satellites_used       = 0;
bool reserved3       = 0;
int c_n0_max       = 0;
float hpa       = 0;
float vpa       = 0;


TinyGPS gps;
char numero_cell[] = "+34756610045";
int estado = 8;//////////////////////////////////////////////
int i = 0;
int j = 0;
String latitud, longitud, mapa, datos;///////////////////////
float lati, longi = 0;
char DAT;
char DAT_dos;
char DAT_GPS;
float flat, flon;
unsigned long age;
char coordenada_gps;
char datosSerial[30];
char clave_gps[] = {'G', 'P', 'S'};
char clave_rele[] = {'R', 'E', 'L', 'E'};
char clave_error[] = {'E', 'R', 'R', 'O', 'R'};


void enableGPS(void)
{
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
}

void disableGPS(void)
{
  modem.sendAT("+SGPIO=0,4,1,0");
  if (modem.waitResponse(10000L) != 1)
  {
    DBG(" SGPIO=0,4,1,0 false ");
  }
  modem.disableGPS();
}

void modemPowerOn()
{
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);
}

void modemPowerOff()
{
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, LOW);
  delay(1500);
  digitalWrite(PWR_PIN, HIGH);
}


void modemRestart()
{
  modemPowerOff();
  delay(3000);
  modemPowerOn();
}

void setup()
{
  SerialMon.begin(115200);
  SerialMon.println("SIMCOM7000G GPS Tracker");
  delay(1000);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  modemPowerOn();

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  SerialMon.println("/**********************************************************/");
  SerialMon.println("Para inicializar la prueba de red, asegúrese de que su GPS");
  SerialMon.println("la antena se ha conectado al puerto GPS de la placa.");
  SerialMon.println("/**********************************************************/\n\n");

  delay(10000);

  if (!modem.testAT())
  {
    SerialMon.println("No se pudo reiniciar el módem, intentando continuar sin reiniciar");
    modemRestart();
    return;
  }

  SerialMon.println("Empezando a posicionar. Asegúrese de ubicarse al aire libre.");
  SerialMon.println("La luz indicadora azul parpadea para indicar el posicionamiento.");

  enableGPS();
  
  float lat,  lon;
    while (1) {
        if (modem.getGPS(&lat, &lon)) {
            Serial.println("The location has been locked, the latitude and longitude are:");
            Serial.print("latitude:"); Serial.println(lat);
            Serial.print("longitude:"); Serial.println(lon);
            break;
        }
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(2000);
    }
 disableGPS();
  while (1) {
        while (SerialAT.available()) {
            SerialMon.write(SerialAT.read());
        }
        while (SerialMon.available()) {
            SerialAT.write(SerialMon.read());
        }
    }
  configuracion_inicial();
{
  SerialMon.println("Sistema Alarma Encendida");
}
  delay(1000);
}

void loop()
{
  SerialMon.println("Esperando recibir mensaje...");
  while (true)
  {
    leer_mensaje();
    activacion_gps();
    solo_lectura_gps();
  }
// Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
      modem.simUnlock(GSM_PIN);
}
}
void configuracion_inicial()
{
  SerialMon.println("Configurando...espere");
  delay(2000);
  SerialAT.print("AT+IPR=115200");
  SerialMon.println("AT+IPR=115200");
  delay(500);
  SerialAT.print("AT+CMGF=1");//Modo texto
  SerialMon.println("AT+CMGF=1");//Modo texto
  delay(500);
  SerialAT.print("AT+CMGR=?");//Activamos codigo para recibir mensajes
  SerialMon.println("AT+CMGR=?");//Activamos codigo para recibir mensajes
  delay(500);
  SerialAT.print("AT+CNMI=2,2,0,0");//Activamos para ver mensajes
  SerialMon.println("AT+CNMI=2,2,0,0");//Activamos para ver mensajes
  delay(500);
  SerialMon.println("Configuracion terminada 1");
  delay(500);
  SerialAT.print("AT+CGNSPWR=1");//Activa el GPS desde el inicio
  SerialMon.println("AT+CGNSPWR=1");//Activa el GPS desde el inicio
  SerialMon.println("Configuracion terminada 2");
  delay(300);
}

void leer_mensaje()
{
salir:
  if (SerialAT.available() > 0)
  {
    char DAT = SerialAT.read();
    if (DAT == '@') //detecta el codigo sms
    { //If arroba
      SerialMon.println("++++++  llego dato  ++++++");
      while (true) //para leer los codigos despues de la arroba
      {
        if (SerialAT.available() > 0)
        { //cierre del segundo if
          char DAT_dos = SerialAT.read(); //@gps enter
          datosSerial[j] = DAT_dos; //almacena y suma caracteres
          j++;
          if (DAT_dos == '\n') //cuando termine de entregar todos los datos dara un enter
            //garantizando el final del codigo
          {
            SerialMon.println ("Lectura correcta del codigo enviado:");
            for (int i = 0; i <= j; i++)
            {
              SerialAT.print(datosSerial[i]);//imprime el codigo guardado en el array
            }
            data_gps();
            data_rele();
            data_error();
            delay(10);

            for (int i = 0; i <= j; i++) //borra los datos del array
            {
              datosSerial[i] == 0; //borro array
              DAT_dos = 0;
              DAT = 0;
            }
            j = 0;
            goto salir;
          }//cierra el /N
        } //cierra 2do if
      } //cierra el while
    } //cierra arroba
  } //cierra SerialMon available
}

void data_gps()
{ //                              G                                 P                                 S
  if (datosSerial[0] == clave_gps[0] && datosSerial[1] == clave_gps[1] && datosSerial[2] == clave_gps[2] || datosSerial[0] == clave_gps[0] && datosSerial[1] == clave_gps[1] && datosSerial[2] == clave_gps[2])
  {
    SerialMon.println("GPS activado");
    activacion_gps();
    //envio_mensaje_gps_coordenada();
    SerialMon.println("lectura de gps enviada");
    SerialMon.print("LAT= ");
    SerialMon.println(lati);
    SerialMon.print("LON= ");
    SerialMon.println(longi);
  }
  else
  SerialMon.println("Codigo incorrecto gps");
}

void data_rele()
{ //                               R                                  E                                  L                                  E
  if (datosSerial[0] == clave_rele[0] && datosSerial[1] == clave_rele[1] && datosSerial[2] == clave_rele[2] && datosSerial[3] == clave_rele[3] || datosSerial[0] == clave_rele[0] && datosSerial[1] == clave_rele[1] && datosSerial[2] == clave_rele[2] && datosSerial[3] == clave_rele[3])
  {
    SerialMon.println("RELE activado");
    rele_encendido(); //envia mensaje al movil
  }
  else
    SerialMon.println("Codigo enviado Rele incorrecto");
  rele_apagado(); //envia mensaje al movil
} 

void data_error()
{ //                                E                                   R                                   R                                   O                                   R
  if (datosSerial[0] == clave_error[0] && datosSerial[1] == clave_error[1] && datosSerial[2] == clave_error[2] && datosSerial[3] == clave_error[3] && datosSerial[4] == clave_error[4] || datosSerial[0] == clave_error[0] && datosSerial[1] == clave_error[1] && datosSerial[2] == clave_error[2] && datosSerial[3] == clave_error[3] && datosSerial[4] == clave_error[4])
  {
    Serial.println("Codigo enviado correcto");
  }
  else
    Serial.println("Codigo enviado incorrecto");
}

void rele_apagado()
{
  SerialAT.println("AT+CMGF=1");
  delay(1000);
  SerialAT.print("AT+CMGS=");
  delay(1000);
  SerialAT.print((char)34);
  SerialAT.print(numero_cell);
  SerialAT.println((char)34);
  delay(1000);
  SerialAT.print("Rele apagado");
  SerialAT.print((char)26);
  Serial.println("Rele apagado");
}

void rele_encendido()
{
  SerialAT.println("AT+CMGF=1");
  delay(1000);
  SerialAT.print("AT+CMGS=");
  delay(1000);
  SerialAT.print((char)34);
  SerialAT.print(numero_cell);
  SerialAT.println((char)34);
  delay(1000);
  SerialAT.print("Rele encendido");
  SerialAT.print((char)26);
  Serial.println("Rele encendido");
  delay(1000);
}

void activacion_gps()
{
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    DBG(" SGPIO=0,4,1,1 false ");
  }
  modem.enableGPS();
  float lat,  lon;
  while (1) {
    if (modem.getGPS(&lat, &lon)) {
      Serial.printf("lat:%f lon:%f\n", lat, lon);
      break;
    } else {
      Serial.print("getGPS ");
      Serial.println(millis());
    }
    delay(2000);
  SerialAT.print("AT+CGNSPWR=1");
  SerialMon.println("AT+CGNSPWR=1");
  delay(400);
  SerialAT.print("AT+CGNSTST=1");
  SerialMon.println("AT+CGNSTST=1");
  delay(400);
  SerialMon.println("Por favor espere que el gps se estabilize y obtenga los datos");
  for (int i = 0; i < 10; i++)
  {
    delay(1000);
    SerialMon.println("Tiempo= ");
    SerialAT.print(i);
  }
  SerialMon.println("Empezando lectura y conversion");
  tyni_gps_leer();
}
}
void tyni_gps_leer()
{
  i = 0;
  while (i < 5)
  {
    i++;
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
      while (SerialAT.available())
      {
        char c = SerialAT.read();
        if (gps.encode(c))
          newData = true;
      }
    }
    if (newData)
    {
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      SerialAT.print("LAT=");
      SerialAT.println(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      lati = flat;
      SerialAT.print("LON=");
      SerialAT.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      longi = flon;
    }
  }
  i = 0;
}

void solo_lectura_gps()
{
  if (SerialAT.available());
}

void envio_mensaje_gps_coordenada()
{
  SerialMon.println("Datos al movil remoto");

  String latitud = String(lati, 6);
  String longitud = String(longi, 6);
  String maps = "https://maps.google.com/maps?q=";
  String datos = maps + latitud + "+" + longitud;
  SerialAT.println(datos);
  SerialMon.print(datos);
  SerialAT.println("AT+CMGF=1");
  delay(1000);
  SerialAT.print("AT+CMGS=");
  delay(1000);
  SerialAT.print((char)34);
  SerialAT.print(numero_cell);
  SerialAT.println((char)34);
  delay(1000);
  SerialAT.print("LAT= ");
  SerialAT.println(lati, 6);
  SerialAT.print("LON= ");
  SerialAT.println(longi, 6);
  SerialAT.print((char)26); //ponemos el simbolo ascii 26 = CTRL+Z
  delay(1000);
  lati, longi = 0; //borro la variable
}
