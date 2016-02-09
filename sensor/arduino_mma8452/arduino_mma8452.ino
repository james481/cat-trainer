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
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "printf.h"

/*
 * Configuration
 */
#define DEBUG true
#define FEATHER32U4 false
#define BAT_VCC_HIGH 4.0
#define BAT_VCC_MED 3.5
#define RADIO_CHANNEL 76
#define RADIO_TIMEOUT 250

// Status LED interval - Watchdog 8 sec cycle count
#define STATUS_FLASH_DELAY 4

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
#define FEATHER32U4_BATTERY_PIN A9

/*
 * MMA8452 Registers
 */
#define ADDR 0x1D
#define STATUS 0x00
#define OUT_X_MSB 0x01
#define OUT_X_LSB 0x02
#define OUT_Y_MSB 0x03
#define OUT_Y_LSB 0x04
#define OUT_Z_MSB 0x05
#define OUT_Z_LSB 0x06
#define SYSMOD 0x0B
#define INT_SOURCE 0x0C
#define WHO_AM_I 0x0D
#define XYZ_DATA_CFG 0x0E
#define HP_FILTER_CUTOFF 0x0F
#define TRANSIENT_CFG 0x1D
#define TRANSIENT_SRC 0x1E
#define TRANSIENT_THS 0x1F
#define TRANSIENT_COUNT 0x20
#define CTRL_REG1 0x2A
#define CTRL_REG2 0x2B
#define CTRL_REG3 0x2C
#define CTRL_REG4 0x2D
#define CTRL_REG5 0x2E

/*
 * Hardware Config
 */

// NRF24L01+ Setup
RF24 radio(GPIO_RF24_CE, GPIO_RF24_CSN);
const uint64_t pipes[2] = { 0xF0F0F0F0F1LL, 0xF0F0F0F0D2LL };
uint8_t sensorId = 0;

/*
 * Interrupt Registers
 */
volatile uint8_t sensorInterrupt = 0;
volatile uint8_t watchdogCount = 0;
volatile bool statusLedActive = false;

/*
 * Data Packet
 */
struct sensorData {
  uint8_t id;
  uint8_t count;
} myData;

/*
 * Setup / Initialize
 */
void setup(void) {
  Serial.begin(115200);

#if DEBUG
  printf_begin();
  Serial.println(F(""));
#endif

  setupRadio();
  setupSensor();

  // Setup LED Pins
  pinMode(GPIO_LED_R, OUTPUT);
  pinMode(GPIO_LED_G, OUTPUT);
  pinMode(GPIO_LED_B, OUTPUT);

  setLedColor(0,0,0);

  delay(250);

  // Disable Interrupts.
  cli();

  // Setup Watchdog to fire interrupt at 8 second intervals.
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  WDTCSR = (1<<WDIE) | (0<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);

  // Setup Timer2 to turn off status LED via interrupt.
  TCCR2A = 0;
  TCCR2B = 0;

  // 256 prescaler
  TCCR2B |= (1 << CS12);
  //
  // enable timer overflow interrupt
  TIMSK2 |= (1 << TOIE1);

  // Enable Interrupts.
  sei();

  delay(250);
}

/*
 * Main Loop
 */
void loop(void) {
#if DEBUG
  Serial.println(F("Running Main Loop."));
#endif

  // Turn off the status LED if it's on.
  if (statusLedActive) {
#if DEBUG
  Serial.println(F("Status LED Disable."));
#endif
    setLedColor(0,0,0);
    statusLedActive = false;
  }

  // Sync to base host and get a sensor id.
  if (sensorId == 0) {
    sensorId = syncSensor();
  }

  if (sensorInterrupt > 0) {
    // Send sensor trigger packet
    sendSensorTrigger();
    sensorInterrupt = 0;
  }

  // Flash the status LED after a certain number of watchdog cycles.
  if (watchdogCount >= STATUS_FLASH_DELAY) {
    enableStatusLed();
    watchdogCount = 0;
  }

#if DEBUG
  // Time for serial debug send.
  delay(50);
#endif

  // Enter Sleep.
  enterSleep();
}

/*
 * Turn on the status LED with a color based on the battery status
 */
void enableStatusLed(void) {
#if DEBUG
  Serial.println(F("Status LED Enable."));
#endif

#if FEATHER32U4
  float measuredvbat = analogRead(FEATHER32U4_BATTERY_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

#if DEBUG
  Serial.print(F("Battery Voltage: "));
  Serial.println(measuredvbat);
#endif

  if (measuredvbat > BAT_VCC_HIGH) {
    setLedColor(0, 255, 0);
  } else if (measuredvbat > BAT_VCC_MED) {
    setLedColor(255, 255, 66);
  } else {
    setLedColor(255, 0, 0);
  }

#else
  setLedColor(255, 255, 255);
#endif

  statusLedActive = true;
}

/*
 * Enter sleep mode and wait for the next interrupt.
 */
void enterSleep(void) {
  // We need Timer1 active to turn off the LED.
  if (statusLedActive) {
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  } else {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    power_all_disable();
  }

  sleep_enable();

  // ZZZZZZ...
  sleep_mode();

  // Wake up here.
  sleep_disable();
  power_all_enable();
}

/*
 * Setup NRF24L01+ chip.
 */
void setupRadio(void) {

#if DEBUG
  Serial.println(F("Initializing NRF24L01+:"));
#endif

  radio.begin();

  radio.enableDynamicPayloads();
  radio.setAutoAck(1);

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(15,15);

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);

#if DEBUG
  radio.printDetails();
  delay(1000);
#endif

  // Power Down until something happens.
  radio.powerDown();
}

/*
 * Setup MMA8452 Sensor (0.5G Z axis transient motion interrupt).
 */
void setupSensor(void) {
#if DEBUG
  Serial.print(F("Initializing MMA8452 Sensor:"));
#endif
  // TODO Multiple sensor setup
  uint8_t address = ADDR;
  uint8_t intPin = GPIO_MMA_IRQ1;

  // Check who am i = 0x2A
  byte who_am_i = readRegister(address, WHO_AM_I);
  if (who_am_i != 0x2A) {

#if DEBUG
    Serial.println(F(" ERROR: Unable to connect to MMA8452."));
#endif
    return;
  }

  // Set Standby mode for setting modes /
  // 100 Hz Output Data Rate / 6.25 Hz Sleep Data Rate
  writeRegister(address, CTRL_REG1, 0x98);

  // Set 8G Scale
  byte cfg = readRegister(address, XYZ_DATA_CFG);
  // Mask Scale bits
  cfg &= 0xFC;
  cfg |= 0x02;
  writeRegister(address, XYZ_DATA_CFG, cfg);

  // Set Transient Event Detection on Z (0b100)
  writeRegister(address, TRANSIENT_CFG, 0x04);

  // Set Transient Event Threshold to 0.5G - (0.5/0.063 = ~8)
  writeRegister(address, TRANSIENT_THS, 0x08);

  // Disable Transient Debounce
  writeRegister(address, TRANSIENT_COUNT, 0x0);

  // Set Transient Interrupt Enabled, point to INTERRUPT_1
  writeRegister(address, CTRL_REG4, 0x20);
  writeRegister(address, CTRL_REG5, 0x20);

  // Set Active (Ready) Mode
  byte ready = readRegister(address, CTRL_REG1);
  writeRegister(address, CTRL_REG1, ready | 0x01);

  // Attach Interrupt
  attachInterrupt(digitalPinToInterrupt(intPin), sensorIsr, FALLING);

#if DEBUG
  Serial.println(F(" Complete."));
#endif
}

/*
 * Write a value to a MMA8452 register.
 */
void writeRegister(uint8_t address, byte reg, byte val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

/*
 * Read a value from a MMA8452 register.
 */
byte readRegister(uint8_t address, byte reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (byte) 1);

  while(!Wire.available()) ;
  return(Wire.read());
}

/*
 * Send a packet to the base indicating the sensor has fired.
 */
void sendSensorTrigger(void) {
  unsigned long started = millis();
  bool timeout = false;

  myData.id = sensorId;
  myData.count = sensorInterrupt;

#if DEBUG
  Serial.println(F("Sensor triggered, sending."));
#endif

  setLedColor(255, 0, 0);
  statusLedActive = true;

  radio.powerUp();

  if (!radio.write(&myData, sizeof(myData))) {
    setLedColor(255, 255, 66);

#if DEBUG
    Serial.println(F("Send failed."));
#endif

  }

  // Listen for a reply or until we timeout.
  radio.startListening();

  while (!radio.available() && !timeout) {
    if ((millis() - started) > RADIO_TIMEOUT) {
      timeout = true;
    }
  }

  if (timeout) {
    // Flash yellow
    setLedColor(255, 255, 66);

#if DEBUG
    Serial.println(F("Radio Timeout."));
#endif

  } else {
    uint8_t len = radio.getDynamicPayloadSize();
    if (len) {
      radio.read(&myData, len);
    }

#if DEBUG
    Serial.println("Radio got response.");
#endif

  }

  radio.stopListening();

  // Wait a bit before flashing the status LED.
  if (watchdogCount) {
    watchdogCount--;
  }

  // Power the radio back down.
  radio.powerDown();
}

/*
 * Convenience method to set the LED to a RGB color (or off).
 */
void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  analogWrite(GPIO_LED_R, 255 - red);
  analogWrite(GPIO_LED_G, 255 - green);
  analogWrite(GPIO_LED_B, 255 - blue);
}

/*
 * Sync the sensor to the base / host and get a sensor id.
 */
uint8_t syncSensor(void) {
#if DEBUG
  Serial.println(F("Syncing sensor ID."));
#endif
  return(1);
}

/*
 * Sensor Interrupt - Wakeup and handle an interrupt from the sensor(s).
 */
void sensorIsr(void) {
  sensorInterrupt++;
}

/*
 * Watchdog Interrupt - Wakeup every 8 seconds and increment the watchdog
 * counter.
 */
ISR(WDT_vect) {
  watchdogCount++;
}

/*
 * Timer2 Interrupt - Wakeup a short time after the status led has turned on
 * and turn it off again.
 */
ISR(TIMER2_OVF_vect) {
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
