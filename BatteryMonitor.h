#ifndef BatteryMonitor_h
#define BatteryMonitor_h

#include <Arduino.h>  // Inclua a biblioteca padrão do Arduino
#include <BQ24195.h>  // Inclua a biblioteca do PMIC BQ24195

class BatteryMonitor {
  public:
    // Construtor
    BatteryMonitor(int R1, int R2, float batteryFullVoltage, float batteryEmptyVoltage, float batteryCapacity);
    
    // Métodos
    void begin();                 // Inicializa o monitor de bateria
    int updateBatteryStatus(); // Atualiza e retorna o status da bateria como String

  private:
    // Atributos
    int R1, R2;
    float batteryFullVoltage, batteryEmptyVoltage, batteryCapacity;
    float rawADC, voltADC, voltBat, max_Source_voltage;

    // Métodos privados
    void calculateVoltage();      // Calcula a tensão da bateria
    int getBatteryStatus();    // Retorna o status da bateria
};

#endif
