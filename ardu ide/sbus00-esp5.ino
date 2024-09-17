#include <Arduino.h>

#define SBUS_RX_PIN 16 // GPIO16 (RX2)
#define SBUS_TX_PIN 17 // GPIO17 (TX2)
#define SBUS_BAUDRATE 100000
#define SBUS_CONFIG SERIAL_8E2
#define SBUS_FRAME_SIZE 25
#define SBUS_NUM_CHANNELS 16

void setup() {
  pinMode(SBUS_RX_PIN, INPUT_PULLUP);
  pinMode(SBUS_TX_PIN, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(SBUS_BAUDRATE, SBUS_CONFIG, SBUS_RX_PIN, SBUS_TX_PIN);
  Serial2.setRxInvert(true);  
  Serial.println("Setup completed. Waiting for SBUS frames...");
  
  // Добавим проверку инициализации UART2
  if (!Serial2) {
    Serial.println("Failed to initialize UART2");
  } else {
    Serial.println("UART2 initialized successfully");
  }
}

// Функция для инверсии всех битов в байте
uint8_t invertByte(uint8_t byte) {
  return ~byte;
}

void loop() {
  static uint8_t sbusFrame[SBUS_FRAME_SIZE];
  static int sbusIndex = 0;
  static unsigned long lastFrameTime = 0;

  while (Serial2.available()) {
    uint8_t incomingByte = Serial2.read();
    // uint8_t writeByte = incomingByte;
    uint8_t writeByte = invertByte(incomingByte);
    
    if (sbusIndex == 0 && incomingByte != 0x0F) {
      continue;
    }

    sbusFrame[sbusIndex] = incomingByte;
    sbusIndex++;

    if (sbusIndex != SBUS_FRAME_SIZE) {
      Serial2.write(&writeByte, 1);
    } else {
      writeByte = 0x00;
      // writeByte = invertByte(0x00);
      Serial2.write(&writeByte, 1);

      for (int i = 0; i < SBUS_FRAME_SIZE; i++) {
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(sbusFrame[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      sbusIndex = 0;
      lastFrameTime = millis();
    }
  }

  // Проверка на отсутствие кадров
  if (millis() - lastFrameTime > 1000) {
    Serial.println("No SBUS frames received in the last second");
    lastFrameTime = millis();
  }
}
