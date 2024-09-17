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

// Функция для декодирования значений каналов SBUS
void decodeSbusFrame(uint8_t *sbusFrame, uint16_t *channels) {
  channels[0]  = ((sbusFrame[1]    |sbusFrame[2]<<8)                 & 0x07FF);
  channels[1]  = ((sbusFrame[2]>>3 |sbusFrame[3]<<5)                 & 0x07FF);
  channels[2]  = ((sbusFrame[3]>>6 |sbusFrame[4]<<2 |sbusFrame[5]<<10)  & 0x07FF);
  channels[3]  = ((sbusFrame[5]>>1 |sbusFrame[6]<<7)                 & 0x07FF);
  channels[4]  = ((sbusFrame[6]>>4 |sbusFrame[7]<<4)                 & 0x07FF);
  channels[5]  = ((sbusFrame[7]>>7 |sbusFrame[8]<<1 |sbusFrame[9]<<9)   & 0x07FF);
  channels[6]  = ((sbusFrame[9]>>2 |sbusFrame[10]<<6)                & 0x07FF);
  channels[7]  = ((sbusFrame[10]>>5|sbusFrame[11]<<3)                & 0x07FF);
  channels[8]  = ((sbusFrame[12]   |sbusFrame[13]<<8)                & 0x07FF);
  channels[9]  = ((sbusFrame[13]>>3|sbusFrame[14]<<5)                & 0x07FF);
  channels[10] = ((sbusFrame[14]>>6|sbusFrame[15]<<2|sbusFrame[16]<<10) & 0x07FF);
  channels[11] = ((sbusFrame[16]>>1|sbusFrame[17]<<7)                & 0x07FF);
  channels[12] = ((sbusFrame[17]>>4|sbusFrame[18]<<4)                & 0x07FF);
  channels[13] = ((sbusFrame[18]>>7|sbusFrame[19]<<1|sbusFrame[20]<<9)  & 0x07FF);
  channels[14] = ((sbusFrame[20]>>2|sbusFrame[21]<<6)                & 0x07FF);
  channels[15] = ((sbusFrame[21]>>5|sbusFrame[22]<<3)                & 0x07FF);
}

void printChannels(uint8_t *sbusFrame) {
  static uint16_t channels[SBUS_NUM_CHANNELS];
  decodeSbusFrame(sbusFrame, channels);

  for (int i = 0; i < SBUS_NUM_CHANNELS; i++) {
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(channels[i]);
    Serial.print(" ");
  }
  Serial.println();
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
      //sbusFrame[SBUS_FRAME_SIZE - 1] = 0x00;

      printChannels(sbusFrame);
      for (int i = 0; i < SBUS_FRAME_SIZE; i++) {
        Serial.print(i + 1);
        Serial.print(":");
        Serial.print(sbusFrame[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      Serial2.write(sbusFrame, SBUS_FRAME_SIZE);
      
      sbusIndex = 0;
      lastFrameTime = millis();
    }
  }

  // Проверка на отсутствие кадров
  if (millis() - lastFrameTime < 1000) {
    Serial.println("No SBUS frames received in the last second");
    lastFrameTime = millis();
  }
}
