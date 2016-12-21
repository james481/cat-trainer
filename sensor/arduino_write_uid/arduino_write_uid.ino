/*
 * arduino_write_uid.ino
 *
 * Write Arduino-based sensor EEPROM storage with unique sensor ID.
 */

#include <EEPROM.h>
#include <SPI.h>
#include "RF24.h"
#include "printf.h"
#include "ct_sensor.h"

/*
 * Configuration
 */
#define EEPROM_ADDR 0
#define SENSOR_NAME "AF8452PCB"
#define SENSOR_ID 2

/*
 * ID Storage (EEPROM)
 */
SensorUid myUid;

/*
 * Setup / Initialize
 */
void setup(void) {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  printf_begin();

  sensorUid myUid = {
    SENSOR_NAME,
    SENSOR_ID
  };

  printf_P(PSTR("Writing myUid type %s id %d\r\n"), myUid.stype, myUid.uid);

  EEPROM.put(EEPROM_ADDR, myUid);

  printf_P(PSTR("Done\r\n"));
}

/*
 * Main Loop
 */
void loop(void) {
  // Empty
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
