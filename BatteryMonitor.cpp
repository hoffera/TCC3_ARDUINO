#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(int R1, int R2, float batteryFullVoltage, float batteryEmptyVoltage, float batteryCapacity) 
  : R1(R1), R2(R2), batteryFullVoltage(batteryFullVoltage), batteryEmptyVoltage(batteryEmptyVoltage), batteryCapacity(batteryCapacity) {
}

void BatteryMonitor::begin() {
    Serial.begin(9600);               
    analogReference(AR_DEFAULT);      
    analogReadResolution(12);         

    PMIC.begin();                                              
    PMIC.enableBoostMode();                                     
    PMIC.setMinimumSystemVoltage(batteryEmptyVoltage);          
    PMIC.setChargeVoltage(batteryFullVoltage);                  
    PMIC.setChargeCurrent(batteryCapacity / 2);                 
    PMIC.enableCharge();                                        
    max_Source_voltage = (3.3 * (R1 + R2)) / R2;
}

void BatteryMonitor::calculateVoltage() {
    rawADC = analogRead(ADC_BATTERY);                     
    voltADC = rawADC * (3.3 / 4095.0);                    
    voltBat = voltADC * (max_Source_voltage / 3.3);       
}

int BatteryMonitor::getBatteryStatus() {
    int new_batt = (voltBat - batteryEmptyVoltage) * (100) / (batteryFullVoltage - batteryEmptyVoltage);

    if (new_batt <= 20) {
        return 1;
    } else if (new_batt > 20 && new_batt <= 50) {
        return 2;
    } else if (new_batt > 50 && new_batt <= 80) {
        return 3;
    } else {
        return 4;
    }
}

int BatteryMonitor::updateBatteryStatus() {
    calculateVoltage();
    return getBatteryStatus();
}
