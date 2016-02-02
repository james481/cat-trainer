/*
 * arduino_mma8452.ino
 *
 * Remote behavior detection node using MMA8452 accelerometer and NRF24L01+ on
 * an Arduino compatible host.
 */

#include <SPI.h>
#include <Wire.h>
#include "nRF24L01.h"
#include "RF24.h"

/*
 * Configuration
 */
#define DEBUG 1
#define FEATHER32U4 0
#define RADIO_CHANNEL 76

/*
 * GPIO Connections
 */
#define GPIO_RF24_CE 7
#define GPIO_RF24_CSN 8
#define GPIO_MMA_IRQ1 2
#define GPIO_MMA_IRQ2 3
#define GPIO_LED_R 5
#define GPIO_LED_G 6
#define GPIO_LED_B 9

/*
 * MMA8452 Registers
 */
#define ADDR = 0x1D
#define STATUS = 0x00
#define OUT_X_MSB = 0x01
#define OUT_X_LSB = 0x02
#define OUT_Y_MSB = 0x03
#define OUT_Y_LSB = 0x04
#define OUT_Z_MSB = 0x05
#define OUT_Z_LSB = 0x06
#define SYSMOD = 0x0B
#define INT_SOURCE = 0x0C
#define WHO_AM_I = 0x0D
#define XYZ_DATA_CFG = 0x0E
#define HP_FILTER_CUTOFF = 0x0F
#define TRANSIENT_CFG = 0x1D
#define TRANSIENT_SRC = 0x1E
#define TRANSIENT_THS = 0x1F
#define TRANSIENT_COUNT = 0x20
#define CTRL_REG1 = 0x2A
#define CTRL_REG2 = 0x2B
#define CTRL_REG3 = 0x2C
#define CTRL_REG4 = 0x2D
#define CTRL_REG5 = 0x2E

/*
 * Hardware Config
 */

// NRF24L01+ Setup
RF24 radio(GPIO_RF24_CE, GPIO_RF24_CSN);
const uint64_t pipes[2] = { 0xF0F0F0F0F1LL, 0xF0F0F0F0D2LL };
uint8_t sensorId = 0;

/*
 * Setup / Initialize
 */
void setup(void) {
  Serial.begin(115200);
  setupRadio();
  setupSensor();
}

/*
 * Main Loop
 */
void loop(void) {

  // Sync to base host and get a sensor id.
  if (sensorId == 0) {
    sensorId = syncSensor();
  }

  // Go to sleep and wait for an interrupt from the
  // accelerometer.

}

/*
 * Setup NRF24L01+ chip.
 */
void setupRadio(void) {

#if DEBUG == 1
  Serial.print(F("Initializing NRF24L01+:"));
#endif

  radio.begin();

  radio.enableDynamicPayloads();
  radio.setAutoAck(1);

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(RADIO_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(15,15);

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

  // Power Down until something happens.
  radio.powerDown();

#if DEBUG == 1
  radio.printDetails();
  Serial.println(F(" Complete."));
#endif
}

/*
 * Setup MMA8452 Sensor (0.5G Z axis transient motion interrupt).
 */
void setupSensor(void) {
#if DEBUG == 1
  Serial.print(F("Initializing MMA8452 Sensor:"));
#endif

#if DEBUG == 1
  Serial.println(F(" Complete."));
#endif
}

/*
 * Sync the sensor to the base / host and get a sensor id.
 */
uint8_t syncSensor(void) {
  return(1);
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
