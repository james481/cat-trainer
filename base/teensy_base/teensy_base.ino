/*
 * teensy_base.ino
 *
 * Behavior correction base station utilizing DC pump sprayhead and
 * communicating via NRF24L01+ to remote sensor nodes on an Teensy 3.x
 * compatible host.
 */

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Servo.h>
#include <Bounce.h>
#include "ct_sensor.h"

/*
 * Configuration
 */

#define DEBUG true
#define DEBUGOUT Serial1

// NRF24L01
#define GPIO_RF24_CE 14
#define GPIO_RF24_CSN 10

// Buttons
#define GPIO_BTN_ENTER 6
#define GPIO_BTN_DN 7
#define GPIO_BTN_UP 8
#define GPIO_DEBOUNCE 10
#define GPIO_BTN_HOLD_DELAY 500

// Pump / Servo
#define GPIO_SPRAY_PUMP A6
#define GPIO_SPRAY_SERVO A7

#define GPIO_OLED_RESET 17

/*
 * Hardware Config
 */

// NRF24L01+ Setup
RF24 radio(GPIO_RF24_CE, GPIO_RF24_CSN);

// Display Setup
Adafruit_SSD1306 display(GPIO_OLED_RESET);

// Servo Setup
Servo sprayServo;

// Buttons
Bounce buttonEnter(GPIO_BTN_ENTER, GPIO_DEBOUNCE);
Bounce buttonDown(GPIO_BTN_DN, GPIO_DEBOUNCE);
Bounce buttonUp(GPIO_BTN_UP, GPIO_DEBOUNCE);

struct buttonState {
  uint8_t enter;
  uint8_t up;
  uint8_t down;
} btnState;

/*
 * Sensor ID Storage
 */
sensorUid remoteUid;
sensor sensors[3];

/*
 * Spray / Movement / Menu States
 */
bool sprayActive = false;
bool sprayMoving = false;
elapsedMillis sprayTimer;

uint8_t menuActive = 0;
uint8_t menuItemActive = 0;
elapsedMillis menuTimer;

/*
 * Menu Struct / Definitions
 */
struct menuItem {
  char name[20];
  char action[5];
  menuItem *next;
  menuItem *prev;
};

/*
 * Setup / Initialize
 */
void setup(void) {

#if DEBUG
  DEBUGOUT.begin(9600);
  printf_begin();
  DEBUGOUT.println(F(""));
  delay(1000);
#endif

  setupRadio();

  // Setup Buttons
  pinMode(GPIO_BTN_ENTER, INPUT);
  pinMode(GPIO_BTN_DN, INPUT);
  pinMode(GPIO_BTN_UP, INPUT);

  // Setup Spray Pump
  pinMode(GPIO_SPRAY_PUMP, OUTPUT);
  analogWrite(GPIO_SPRAY_PUMP, 0);

  // Setup Servo
  sprayServo.attach(GPIO_SPRAY_SERVO);
  sprayServo.write(90);

#if DEBUG
  DEBUGOUT.println(F("Base Ready."));
#endif

  // Setup Display / Show Splash
  setupDisplay();
}

/*
 * Main Loop
 */
void loop(void) {
  // Check Input Button Status
  checkButtons();

  // Check Menu / Timeout Status
  // Check Registered Sensors / Update Status
  // Check Servo Moving Status
  // Check Water Level Status
  // Check Spray Start / Stop Status
  // Check Radio For Packets
  // Update EEPROM Sensor Storage
  // Update Status Display

#if DEBUG
  DEBUGOUT.println(F("LOOP"));
  delay(1000);
#endif
}

/*
 * Check Button States
 */
void checkButtons(void) {
  bool pushed = false;

  // Check if enter button pressed
  buttonEnter.update();
  if (buttonEnter.fallingEdge()) {
    pushed = true;
    menuTimer = 0;
  } else if (buttonEnter.risingEdge()) {
    btnState.enter = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
  } else {
    btnState.enter = 0;
  }

  // Check if direction buttons pressed (if menu is active)
  if (!pushed && menuActive) {
    buttonUp.update();
    if (buttonUp.fallingEdge()) {
      pushed = true;
      menuTimer = 0;
    } else if (buttonUp.risingEdge()) {
      btnState.up = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
    } else {
      btnState.up = 0;
    }
  }

  if (!pushed && menuActive) {
    buttonDown.update();
    if (buttonDown.fallingEdge()) {
      pushed = true;
      menuTimer = 0;
    } else if (buttonDown.risingEdge()) {
      btnState.down = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
    } else {
      btnState.down = 0;
    }
  }
}

/*
 * Setup Display / Show Splash Screen
 */
void setupDisplay(void) {
#if DEBUG
  DEBUGOUT.println(F("Initializing Display."));
#endif

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Cat");
  display.println("Trainer");
  display.println("");
  display.setTextSize(1);
  display.println("v0.1");
  display.display();

  delay(2000);
}

/*
 * Setup NRF24L01+ chip.
 */
void setupRadio(void) {

#if DEBUG
  DEBUGOUT.println(F("Initializing NRF24L01+:"));
#endif

  radio.begin();

  radio.enableDynamicPayloads();
  radio.setAutoAck(1);

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(RADIO_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(15,15);

  radio.openWritingPipe(control_pipes[0]);
  radio.openReadingPipe(1,control_pipes[1]);

#if DEBUG
  radio.printDetails();
  delay(1000);
#endif
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
