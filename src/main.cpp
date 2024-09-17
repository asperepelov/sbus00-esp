#include <Arduino.h>

#define SBUS_RX_PIN 16 // GPIO16 (RX2)
#define SBUS_TX_PIN 17 // GPIO17 (TX2)
#define SBUS_BAUDRATE 100000
#define SBUS_CONFIG SERIAL_8E2
#define SBUS_FRAME_SIZE 25
#define SBUS_NUM_CHANNELS 16

// Массив для хранения значений каналов
uint16_t channels[SBUS_NUM_CHANNELS];

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
    // if ((i + 1) % 4 == 0) Serial.println();
  }
  Serial.println();
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

      // Декодируем каналы
      decodeChannels(sbusFrame);

      // Выводим значения каналов
      printChannels();
      
      // Выводим сырые данные кадра
      // for (int i = 0; i < SBUS_FRAME_SIZE; i++) {
      //   Serial.print(i + 1);
      //   Serial.print(":");
      //   Serial.print(sbusFrame[i], HEX);
      //   Serial.print(" ");
      // }
      // Serial.println();

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
