#define GPS_Serial_Baud 9600
#include <MKRGSM.h>
#include <TinyGPS.h>
#include <ArduinoJson.h>
#include <ThingSpeak.h>
#include <cmath>
#define M_PI 3.14159265358979323846
#include "BatteryMonitor.h"

GSM gsmAccess;
GPRS gprs;
GSMClient client;
HardwareSerial& gpsSerial = Serial1;
const char apn[] = "kore.br";
const char gsmPin[] = "3636";
const char GPRS_LOGIN[] = "kore";
const char GPRS_PASSWORD[] = "kore";
const char* apiKey = "5VLHEQBMZEFPHP0G";
const char* server = "api.thingspeak.com";
const int port = 80;
TinyGPS gps;

float flat, flon, flat2, flon2;
const float MAX_DISTANCE = 1.0e6;  // Valor alto para distância "infinita"
float minDistance = MAX_DISTANCE;  // Variável para armazenar a menor distância
const int ledPin = 6;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long previousMillis = 0;  // Para armazenar o último tempo que a função 'send()' foi chamada
long interval = 10000;             // Intervalo de 10 segundos
unsigned long lastGPSTime = 0;     // Última vez que tentamos ler o GPS
const long gpsInterval = 500;      // Intervalo para leitura de GPS
unsigned long lastSearchTime = 0;  // Última vez que tentamos encontrar a localização
const long searchInterval = 1000;  // Intervalo para encontrar a menor distância

//********BATERIA*******
BatteryMonitor batteryMonitor(330000, 1000000, 4.2, 3.5, 2.1);
//********BATERIA*******


void setupGSM() {
  Serial.print(".");
  Serial.print(".");
  Serial.println(".");
  bool connected = false;
  while (!connected) {
    if ((gsmAccess.begin(gsmPin) == GSM_READY) && (gprs.attachGPRS(apn, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY)) {
      connected = true;
      Serial.println("connected");
      ThingSpeak.begin(client);
    } else {
      Serial.println("Not connected");
      delay(1000);
    }
  }
  delay(1000);
}

float readTSData(int field) {
  float data;
  Serial.println("Recebendo atualizacao do app");
  data = ThingSpeak.readFloatField(2677228, field, "DI4XDFM9FPB5D3CE");
  return data;
}

void writeTSData(float data1, float data2) {
  int batteryStatus = batteryMonitor.updateBatteryStatus();
  Serial.println("Enviando dados para o ThingSpeak");
  
  // Envia o dado para o field1
  ThingSpeak.setField(1, data1);
  
  // Envia o dado para o field2
  ThingSpeak.setField(2, data2);

  // Enviando bateria para field3
  ThingSpeak.setField(3, batteryStatus);
  
  // Escreve os dados para o canal
  int response = ThingSpeak.writeFields(2293050, "5VLHEQBMZEFPHP0G");
  
  if(response == 200) {
    Serial.println("Dados enviados com sucesso!");
  } else {
    Serial.println("Falha ao enviar os dados. Código de erro: " + String(response));
  }
}

void writeTSDataTest() {
  int batteryStatus = batteryMonitor.updateBatteryStatus();
  Serial.println("Enviando bateria para o ThingSpeak");
  
ThingSpeak.setField(3, batteryStatus);
  
  // Escreve os dados para o canal
  int response = ThingSpeak.writeFields(2293050, "5VLHEQBMZEFPHP0G");
  
  if(response == 200) {
    Serial.println("Dados enviados com sucesso!");
  } else {
    Serial.println("Falha ao enviar os dados. Código de erro: " + String(response));
  }
}


void sendDataToThingSpeak(float latitude, float longitude, float distance) {
  String data = "field1=" + String(longitude, 6) + "&field2=" + String(latitude, 6) + "&field3=" + String(distance, 6);

  if (client.connect(server, port)) {
    client.print("POST /update HTTP/1.1\r\n");
    client.print("Host: api.thingspeak.com\r\n");
    client.print("Connection: close\r\n");
    client.print("X-THINGSPEAKAPIKEY: " + String(apiKey) + "\r\n");
    client.print("Content-Type: application/x-www-form-urlencoded\r\n");
    client.print("Content-Length: ");
    client.print(data.length());
    client.print("\r\n\r\n");
    client.print(data);

    Serial.println("Enviando dados para o ThingSpeak...");

    delay(1000);

    while (client.available()) {
      char c = client.read();
      Serial.print(c);
    }

    client.stop();
    Serial.println("\nDados enviados com sucesso para o ThingSpeak.");
  } else {
    Serial.println("Falha na conexão com o ThingSpeak.");
  }
}

float degreesToRadians(float degrees) {
  return degrees * (M_PI / 180.0);
}

float distanceBetweenEarthCoordinates(float lat1, float lon1, float lat2, float lon2) {
  long earthRadiusMeters = 6371000;
  float dLat = degreesToRadians(lat2 - lat1);
  float dLon = degreesToRadians(lon2 - lon1);
  lat1 = degreesToRadians(lat1);
  lat2 = degreesToRadians(lat2);
  float a = sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return earthRadiusMeters * c;
}

bool isGSMConnected() {
  return gsmAccess.status() == GSM_READY;
}
void disconnectGSM() {
  // Termina a conexão GSM
  gsmAccess.shutdown();
  Serial.println("Conexão GSM terminada");

  if (gprs.detachGPRS() == GPRS_READY) {
    Serial.println("GPRS desconectado");
  } else {
    Serial.println("Falha ao desconectar o GPRS");
  }
}






void searchLoc() {
  digitalWrite(ledPin, HIGH);

  bool newData = false;
  float sumLat = 0.0, sumLon = 0.0;
  int count = 0;
  float minLat = 0.0, minLon = 0.0;

  // Armazena o tempo inicial da pesquisa
  unsigned long startTime = millis();

  Serial.println("Iniciando coleta de dados GPS");

  while (millis() - startTime < 5000) {
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        unsigned long age;
        float lat, lon;
        gps.f_get_position(&lat, &lon, &age);

        if (lat != TinyGPS::GPS_INVALID_F_ANGLE && lon != TinyGPS::GPS_INVALID_F_ANGLE) {
          sumLat += lat;
          sumLon += lon;
          count++;

          Serial.print("LAT= ");
          Serial.print(lat, 6);
          Serial.print("\tLON= ");
          Serial.print(lon, 6);
          Serial.println();
        }
      }
    }
  }

  if (count > 0) {
    // Calcula a média das coordenadas
    float avgLat = sumLat / count;
    float avgLon = sumLon / count;

    Serial.println("Coleta concluída");
    Serial.print("Média LAT: ");
    Serial.print(avgLat, 6);
    Serial.print("\tMédia LON: ");
    Serial.println(avgLon, 6);

    // Envia dados para o ThingSpeak
    writeTSData(avgLat,avgLon);  // Não estamos usando a distância mínima aqui
  } else {
    Serial.println("Nenhum dado GPS válido coletado.");
  }
}






void sendData() {
  
  searchLoc();
  
  interval = readTSData(1);
  Serial.print("interval: ");
  Serial.println(interval);
}


void setup() {
  Serial.begin(9600);
  gpsSerial.begin(GPS_Serial_Baud);
  pinMode(ledPin, OUTPUT);
  setupGSM();
  batteryMonitor.begin(); 
  startMillis = millis();  //initial start time
}


void loop() {
  
  if (isGSMConnected()) {

  currentMillis = millis();
  if (currentMillis - startMillis >= interval) {
    // setupGSM(); // Configura o GSM
    Serial.println(startMillis);
    // Verifica se a conexão GSM foi estabelecida
    if (isGSMConnected()) {
     
     writeTSData(-26.97244052040394, -48.63424548179478);

    //  sendData();
    //  disconnectGSM();
    } else {
      Serial.println("Erro: Timeout de conexão GSM");
    }
    startMillis = currentMillis;
  }}
  else{
    Serial.println("GSM DESCONNECTED");
    setupGSM();
  }
}
