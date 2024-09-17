#include <Arduino.h>
#include <SBUS.h>

#define SBUS_RX_PIN 16 // GPIO16 (RX2)
#define SBUS_TX_PIN 17 // GPIO17 (TX2)

SBUS sbus(Serial2);

void setup() {
  Serial.begin(115200); // Для отладки
  Serial.println("Starting SBUS setup...");
  
  sbus.begin(SBUS_RX_PIN, SBUS_TX_PIN, true); // true для инвертированного SBUS
  
  Serial.println("SBUS setup completed.");
}

void loop() {
  static unsigned long lastFrameTime = 0;

  if (sbus.read()) {
    // SBUS кадр получен
    Serial.println("SBUS frame received");

    // Здесь вы можете обработать данные SBUS
    // Например, получить значения каналов:
    for (int i = 0; i < 16; i++) {
      Serial.print("Channel ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(sbus.getChannel(i));
    }

    // Модифицируйте данные, если необходимо
    // Например, изменим значение первого канала
    uint16_t channels[16];
    for (int i = 0; i < 16; i++) {
      channels[i] = sbus.getChannel(i);
    }
    channels[0] = 1500; // Устанавливаем первый канал в среднее положение

    // Отправляем модифицированный кадр
    sbus.write(channels);

    lastFrameTime = millis();
  }

  // Проверка на отсутствие кадров
  if (millis() - lastFrameTime > 1000) {
    Serial.println("No SBUS frames received in the last second");
    lastFrameTime = millis();
  }
}