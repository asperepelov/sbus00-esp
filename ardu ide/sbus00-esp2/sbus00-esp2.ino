#include <SBUS.h>

// Определение пинов
#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17

SBUS sbus(Serial2);

// Массив для хранения данных каналов
uint16_t channels[16];
// Переменные для хранения данных failsafe и lost frame
bool failsafe;
bool lostFrame;

void setup() {
  // Инициализация последовательного порта для отладки
  Serial.begin(115200);
  
  // Инициализация SBUS
  sbus.Begin(SBUS_RX_PIN, SBUS_TX_PIN);
  Serial.println("SBUS initialized");
}

void loop() {
  // Чтение данных SBUS
  if (sbus.Read()) {
    sbus.getChannels(channels);
    failsafe = sbus.getFailsafe();
    lostFrame = sbus.getLostFrame();

    // Вывод значений каналов
    for (int i = 0; i < 16; i++) {
      Serial.print("Канал ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(channels[i]);
    }
    Serial.print("Failsafe: ");
    Serial.println(failsafe ? "Активен" : "Не активен");
    Serial.print("Lost Frame: ");
    Serial.println(lostFrame ? "Потерян" : "Не потерян");
    delay(1000);
  }
}
