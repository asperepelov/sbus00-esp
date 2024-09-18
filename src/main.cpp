#include <Arduino.h>
#include <HardwareSerial.h>
#include "sbus.h"

#define SBUSIN_RX_PIN 16 // GPIO16 (RX2)
#define SBUSIN_TX_PIN 17 // GPIO17 (TX2)
#define SBUSOUT_RX_PIN 12 // GPIO12 
#define SBUSOUT_TX_PIN 13 // GPIO13
#define SBUS_BAUDRATE 100000
#define SBUS_CONFIG SERIAL_8E2
#define SBUS_FRAME_SIZE 25
#define SBUS_NUM_CHANNELS 16

/* Выходной SBUS */
bfs::SbusTx sbus_tx(&Serial1, SBUSOUT_RX_PIN, SBUSOUT_TX_PIN, true);
bfs::SbusData data;

// Массив для хранения значений каналов
uint16_t channels[SBUS_NUM_CHANNELS];

// Декодирование каналов SBUS
void decodeChannels(uint8_t* frame) {
  channels[0]  = ((frame[1]    |frame[2]<<8)                 & 0x07FF);
  channels[1]  = ((frame[2]>>3 |frame[3]<<5)                 & 0x07FF);
  channels[2]  = ((frame[3]>>6 |frame[4]<<2 |frame[5]<<10)   & 0x07FF);
  channels[3]  = ((frame[5]>>1 |frame[6]<<7)                 & 0x07FF);
  channels[4]  = ((frame[6]>>4 |frame[7]<<4)                 & 0x07FF);
  channels[5]  = ((frame[7]>>7 |frame[8]<<1 |frame[9]<<9)    & 0x07FF);
  channels[6]  = ((frame[9]>>2 |frame[10]<<6)                & 0x07FF);
  channels[7]  = ((frame[10]>>5|frame[11]<<3)                & 0x07FF);
  channels[8]  = ((frame[12]   |frame[13]<<8)                & 0x07FF);
  channels[9]  = ((frame[13]>>3|frame[14]<<5)                & 0x07FF);
  channels[10] = ((frame[14]>>6|frame[15]<<2|frame[16]<<10)  & 0x07FF);
  channels[11] = ((frame[16]>>1|frame[17]<<7)                & 0x07FF);
  channels[12] = ((frame[17]>>4|frame[18]<<4)                & 0x07FF);
  channels[13] = ((frame[18]>>7|frame[19]<<1|frame[20]<<9)   & 0x07FF);
  channels[14] = ((frame[20]>>2|frame[21]<<6)                & 0x07FF);
  channels[15] = ((frame[21]>>5|frame[22]<<3)                & 0x07FF);
}

void printChannels() {
  for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
    Serial.print("C");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(channels[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  pinMode(SBUSIN_RX_PIN, INPUT_PULLUP);
  pinMode(SBUSIN_TX_PIN, OUTPUT);
  Serial.begin(115200); // Терминал
  // Обработка входного SBUS
  Serial2.begin(SBUS_BAUDRATE, SBUS_CONFIG, SBUSIN_RX_PIN, SBUSIN_TX_PIN);
  Serial2.setRxInvert(true);
  
  // Инициализация SBUS для выхода
  sbus_tx.Begin();
  
  Serial.println("Setup completed. Waiting for SBUS frames...");
  
  // Добавим проверку инициализации UART2
  if (!Serial2) {
    Serial.println("Failed to initialize UART2");
  } else {
    Serial.println("UART2 initialized successfully");
  }
}

void loop() {
  static uint8_t sbusFrame[SBUS_FRAME_SIZE];
  static int sbusIndex = 0;
  static unsigned long lastFrameTime = 0;

  while (Serial2.available()) {
    uint8_t incomingByte = Serial2.read();
    
    if (sbusIndex == 0 && incomingByte != 0x0F) {
      continue;
    }

    sbusFrame[sbusIndex] = incomingByte;
    sbusIndex++;

    if (sbusIndex == SBUS_FRAME_SIZE) {
      // Декодируем каналы
      decodeChannels(sbusFrame);

      // Отправляем данные через SBUS
      for (size_t i = 0; i < SBUS_NUM_CHANNELS; i++) {
        data.ch[i] = channels[i];
      }
      sbus_tx.data(data);
      sbus_tx.Write();

      // Выводим значения каналов
      printChannels();
      
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
