#include <Arduino.h>
#include <Bounce2.h>

#define SERIAL_DEBUG true
#define WATCHDOG_TIMEOUT 5000

struct SensorConfig {
  uint8_t inputPin;
  uint8_t relayPin;
  const char* sensorName;
  unsigned long triggerTime;
  unsigned long alarmDuration;
  bool useInvertedLogic;
  bool isAnalog;
  float threshold;
};

Bounce resetDebouncer;

SensorConfig sensors[] = {
  {2, 8, "Vibração", 2000, 30000, false, false, 0},
  {4, 9, "Ruído", 3000, 2000, false, false, 0}
};

const uint8_t relayMainPins[] = {7, 10};
bool systemArmed = true;
unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 100;

void initWatchdog();
void resetWatchdog();
void activateAlarm();
void deactivateAlarm();
void checkSensors();
void handleSerialCommands();
void saveEventToLog(const char* event);

void setup() {
  if(SERIAL_DEBUG) Serial.begin(115200);
  
  //Inicialização

  initWatchdog();
  initSensors();
  initRelays();
  initResetButton();

  saveEventToLog("Sistema Iniciado");
}

// ============= LOOP PRINCIPAL =============
void loop() {
  resetWatchdog(); 
  // Verificação periódica dos sensores
  if(millis() - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
    lastSensorCheck = millis();
    checkSensors();
  }

  // Atualização do debouncer do botão
  resetDebouncer.update();
  
  // Verificação do botão de reset
  if(resetDebouncer.fell()) {
    deactivateAlarm();
    saveEventToLog("Alarme resetado manualmente");
  }

  // Comunicação serial (debug/comandos)
  if(SERIAL_DEBUG) handleSerialCommands();
}

// ============= FUNÇÕES AVANÇADAS =============

void initWatchdog() {
  // Configuração do watchdog para reinício automático em caso de travamento
  #ifndef __AVR__
  if(WDTCSR & _BV(WDE)) {
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDP2) | _BV(WDP1) | _BV(WDE); 
  }
  #endif
}

void resetWatchdog() {
  #ifndef __AVR__
  __asm__ __volatile__ ("wdr");
  #endif
}

void initSensors() {
  for(auto &sensor : sensors) {
    pinMode(sensor.inputPin, sensor.isAnalog ? INPUT : INPUT_PULLUP);
  }
}

void initRelays() {
  for(auto pin : relayMainPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  for(auto &sensor : sensors) {
    pinMode(sensor.relayPin, OUTPUT);
    digitalWrite(sensor.relayPin, HIGH);
  }
}

void initResetButton() {
  resetDebouncer.attach(6, INPUT_PULLUP);
  resetDebouncer.interval(25);
}

void checkSensors() {
  static bool alarmTriggered = false;
  
  for(auto &sensor : sensors) {
    bool sensorState = sensor.isAnalog ? 
      (analogRead(sensor.inputPin) > sensor.threshold) : 
      digitalRead(sensor.inputPin);
      
    if(sensor.useInvertedLogic) sensorState = !sensorState;
    
    if(sensorState && systemArmed) {
      if(!alarmTriggered) {
        activateAlarm();
        char logMsg[50];
        sprintf(logMsg, "Alarme ativado por %s", sensor.sensorName);
        saveEventToLog(logMsg);
        alarmTriggered = true;
      }
    }
  }
}

void activateAlarm() {
  for(auto pin : relayMainPins) digitalWrite(pin, LOW);
  for(auto &sensor : sensors) digitalWrite(sensor.relayPin, LOW);
}

void deactivateAlarm() {
  for(auto pin : relayMainPins) digitalWrite(pin, HIGH);
  for(auto &sensor : sensors) digitalWrite(sensor.relayPin, HIGH);
}

void handleSerialCommands() {
  if(Serial.available()) {
    char cmd = Serial.read();
    switch(cmd) {
      case 'A': systemArmed = !systemArmed; break;
      case 'S':
        Serial.println("=== STATUS DO SISTEMA ===");
        for(auto &sensor : sensors) {
          Serial.print(sensor.sensorName);
          Serial.print(": ");
          Serial.println(digitalRead(sensor.inputPin) ? "ATIVO" : "INATIVO");
        }
        break;
    }
  }
}

void saveEventToLog(const char* event) {
  if(SERIAL_DEBUG) {
    Serial.print("[LOG] ");
    Serial.print(millis());
    Serial.print(" - ");
    Serial.println(event);
  }
  
}