#define GPS_Serial_Baud 9600
#include <MKRGSM.h>
#include <ArduinoJson.h>
#include <ThingSpeak.h>
#include <cmath>
#define M_PI 3.14159265358979323846
#include "BatteryMonitor.h"

GSM gsmAccess;
GPRS gprs;
GSMClient client;
HardwareSerial& gpsSerial = Serial2;
const char apn[] = "kore.br";
const char gsmPin[] = "3636";
const char GPRS_LOGIN[] = "kore";
const char GPRS_PASSWORD[] = "kore";
const char* apiKey = "5VLHEQBMZEFPHP0G";
const char* server = "api.thingspeak.com";
const int port = 80;

float flat, flon;
const float MAX_DISTANCE = 1.0e6;  // Valor alto para distância "infinita"
const int ledPin = 6;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long previousMillis = 0;  // Para armazenar o último tempo que a função 'send()' foi chamada
long interval = 10000;             // Intervalo de 10 segundos
bool validGPS = false;
//*LIB GPS***
#define FIELD_MAX 20
#define HIGH_QUALITY_TIME 5000  // 5 segundos para gpsQuality > 1
#define LOW_QUALITY_TIME 1000   // 1 segundo para gpsQuality > 0

void ProcessNMEALine(char* s) {
  char* field[FIELD_MAX];
  int f;
  double lat, lon;
  char lat_hemi, lon_hemi;

  if (s[0] != '$')
    return;

  f = 0;
  while (1) {
    field[f++] = s;
    while ((s != 0) && (*s != ',') && (*s != '\0') && (*s != 0x0D) && (*s != 0x0A))
      s++;

    if ((s == 0) || (*s == '\0') || (*s == 0x0D) || (*s == 0x0A) || (f == (FIELD_MAX - 1))) {
      *s = 0;
      field[f] = NULL;
      break;
    }
    *s++ = 0;
  }


  if (strncmp(field[0], "$G", 2) == 0) {
    if (((strncmp(&field[0][3], "GGA", 3) == 0) ||   // GPS (EUA)
         (strncmp(&field[0][1], "LGGA", 4) == 0) ||  // GLONASS (Rússia)
         (strncmp(&field[0][1], "AGGA", 4) == 0) ||  // Galileo (União Europeia)
         (strncmp(&field[0][1], "DGGA", 4) == 0) ||  // BeiDou (China)
         (strncmp(&field[0][1], "ZGGA", 4) == 0))    // QZSS (Japão)
        && (f > 14)) {

      int gpsQuality = atoi(field[6]);
      Serial.print("Qualidade do GPS: ");
      Serial.println(gpsQuality);

      if (gpsQuality > 1 && gpsQuality < 5) {
        Serial.print("validgps:");
        Serial.println(validGPS);
        validGPS = true;
        Serial.print("validgps:");
        Serial.println(validGPS);

        // Latitude e hemisfério
        lat = atof(field[2]);
        lat_hemi = field[3][0];

        // Longitude e hemisfério
        lon = atof(field[4]);
        lon_hemi = field[5][0];

        // Conversão de latitude para graus decimais
        int lat_deg = (int)lat / 100;
        double lat_min = lat - (lat_deg * 100);
        lat = (double)lat_deg + (lat_min / 60.0);
        if (lat_hemi == 'S') lat = -lat;

        // Conversão de longitude para graus decimais
        int lon_deg = (int)lon / 100;
        double lon_min = lon - (lon_deg * 100);
        lon = (double)lon_deg + (lon_min / 60.0);
        if (lon_hemi == 'W') lon = -lon;

        // Atualiza as variáveis globais
        flat = lat;
        flon = lon;

        // Exibe os resultados
        Serial.print("Latitude: ");
        Serial.print(lat, 6);
        Serial.print(", Longitude: ");
        Serial.println(lon, 6);
      }
    }
  }
}

#define LINEMAX 200  // Máximo comprimento da linha permitido

void ProcessStream(uint8_t* Buffer, int Size) {
  static char rx_buffer[LINEMAX + 1];  // Buffer local para armazenar a linha
  static int rx_index = 0;

  while (Size--) {  // Processa os dados disponíveis
    char rx = (char)*Buffer++;

    if ((rx == '\r') || (rx == '\n')) {  // Verifica se é fim de linha
      if (rx_index != 0) {               // A linha tem conteúdo
        rx_buffer[rx_index] = 0;         // Adiciona terminador NUL
        ProcessNMEALine(rx_buffer);
        rx_index = 0;  // Reinicia o ponteiro de conteúdo
      }
    } else {
      if ((rx == '$') || (rx_index == LINEMAX))  // Se houver ressincronização ou estouro
        rx_index = 0;

      rx_buffer[rx_index++] = rx;  // Copia para o buffer e incrementa
    }
  }
}

//***BATERIA**
BatteryMonitor batteryMonitor(330000, 1000000, 4.2, 3.5, 2.1);
//***BATERIA**

void setup() {
  Serial.begin(9600);
  setupGSM();
  Serial1.begin(38400);  // Inicia a comunicação serial com o módulo GPS a 38400 bps
  batteryMonitor.begin();
  startMillis = millis();  // Tempo inicial
}

void setupGSM() {
  Serial.println("Tentando conectar ao GSM...");
  bool connected = false;
  while (!connected) {
    if ((gsmAccess.begin(gsmPin) == GSM_READY) && (gprs.attachGPRS(apn, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
      connected = true;
      Serial.println("GSM conectado.");
      ThingSpeak.begin(client);

    } else {
      Serial.println("Falha na conexão GSM.");
      delay(1000);
    }
  }
}

float readTSData(int field) {
  float data;
  Serial.println("Recebendo atualização do app...");
  data = ThingSpeak.readFloatField(2677228, field, "DI4XDFM9FPB5D3CE");
  return data;
}

void writeTSData(float data1, float data2) {
  int batteryStatus = batteryMonitor.updateBatteryStatus();
  Serial.println("Enviando dados para o ThingSpeak...");

  // Envia os dados
  ThingSpeak.setField(1, data1);
  ThingSpeak.setField(2, data2);
  ThingSpeak.setField(3, batteryStatus);

  int response = ThingSpeak.writeFields(2293050, apiKey);
  if (response == 200) {
    Serial.println("Dados enviados com sucesso!");
  } else {
    Serial.println("Erro ao enviar dados. Código: " + String(response));
  }
}

bool isGSMConnected() {
  return gsmAccess.status() == GSM_READY;
}

void disconnectGSM() {
  gsmAccess.shutdown();
  Serial.println("Conexão GSM terminada.");
  if (gprs.detachGPRS() == GPRS_READY) {
    Serial.println("GPRS desconectado.");
  } else {
    Serial.println("Falha ao desconectar GPRS.");
  }
}

void sendData() {


  // Fica preso até obter um sinal GPS válido
  while (!validGPS) {
    int len = Serial1.available();
    if (len > 0) {  // Lê os dados do GPS
      uint8_t buffer[128];
      int xferlen = len;
      if (len > sizeof(buffer)) xferlen = sizeof(buffer);
      Serial1.readBytes(buffer, xferlen);
      ProcessStream(buffer, xferlen);  // Processa os dados lidos
      if (validGPS == true) {
        writeTSData(flat, flon);
      }
    }
  }


  validGPS = false;
}


void loop() {
  currentMillis = millis();

  if (startMillis == 0 || currentMillis - startMillis >= 15000) {

    while (!isGSMConnected()) {
      setupGSM();
    }
    pinMode(ledPin, OUTPUT);
    gsmAccess.lowPowerMode();
    Serial.println("GSM conectado. Tentando enviar dados...");
    sendData();

    startMillis = currentMillis;

    Serial.println("Colocando GSM em repouso...");
    pinMode(ledPin, INPUT);
  }
}
