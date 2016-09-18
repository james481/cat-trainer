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
#define DEBUGOUT Serial
#define MENU_TIMEOUT 10000

// NRF24L01
#define GPIO_RF24_CE 14
#define GPIO_RF24_CSN 10

// Buttons
#define GPIO_BTN_ENTER 6
#define GPIO_BTN_DN 7
#define GPIO_BTN_UP 8
#define GPIO_DEBOUNCE 10
#define GPIO_BTN_HOLD_DELAY 750

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

// Button States: 0 - None, 1 - Click, 2 - Hold
struct buttonState {
  uint8_t enter = 0;
  uint8_t up = 0;
  uint8_t down = 0;
  bool pushed = false;
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

bool menuActive = false;
elapsedMillis menuTimer;

/*
 * Menu Classes
 */
class MenuItem;
class MenuDisplay;

struct MenuExeEvent {
  MenuItem &item;
};

typedef void (*cb_exe)(MenuExeEvent);

class MenuItem {
  public:
    MenuItem(const char* label, const uint8_t uid) : itemLabel(label), itemUid(uid) {
      parent = child = above = below = 0;
    }

    inline const char* getLabel() const {
      return(itemLabel);
    }

    inline const uint8_t getUid() const {
      return(itemUid);
    }

    inline MenuItem *getAbove() const {
      return(above);
    }

    inline MenuItem *getBelow() const {
      return(below);
    }

    inline MenuItem *getChild() const {
      return(child);
    }

    inline MenuItem *getParent() const {
      return(parent);
    }

    MenuItem &onExe(cb_exe cb) {
      cbExe = cb;
      return(*this);
    }

    MenuItem &setParent(MenuItem &item) {
      parent = &item;
      return(item);
    }

    MenuItem &setChild(MenuItem &item) {

#if DEBUG
      printf_P(
        PSTR("Setting Menu item %s (%d) to child %s (%d).\r\n"),
        this->getLabel(),
        this->getUid(),
        item.getLabel(),
        item.getUid()
      );
#endif

      child = &item;
      item.parent = this;

      return(item);
    }

    MenuItem &addAbove(MenuItem &item) {
      item.below = this;
      above = &item;
      return(item);
    }

    MenuItem &addBelow(MenuItem &item) {
      item.above = this;
      below = &item;
      return(item);
    }

    bool isEqual(MenuItem &item) {
      return(getUid() == item.getUid());
    }

    bool isEqual(MenuItem *item) {
      return(getUid() == item->getUid());
    }

    void execute() {
      if (cbExe) {
        MenuExeEvent ev = { *this };
        (*cbExe)(ev);
      }
    }

  protected:
    const char* itemLabel;
    const uint8_t itemUid;

    MenuItem *above;
    MenuItem *below;
    MenuItem *child;
    MenuItem *parent;

    cb_exe cbExe;
};

class MenuDisplay {
  public:
    MenuDisplay(Adafruit_SSD1306 *dis, MenuItem *rootItem) : oled(dis), root(rootItem) {
      current = root;
      first = root;
    }

    MenuItem getRoot() {
      return(*root);
    }

    MenuItem getCurrent() {
      return(*current);
    }

    MenuItem getFirst() {
      return(*first);
    }

    bool handleButtons(buttonState btns) {
      bool redraw = false;

#if DEBUG
      printf_P(
        PSTR("handleButtons: Current %s (%d).\r\n"),
        current->getLabel(),
        current->getUid()
      );
#endif

      // Handle Enter press / hold
      if (btns.enter == 1) {
        if (current->getChild()) {
          setCurrent(current->getChild());
          findFirst(current);
          redraw = true;
        } else {
          current->execute();
        }
      } else if (btns.enter == 2) {
        if (current->getParent()) {
          setCurrent(current->getParent());
          findFirst(current);
          redraw = !current->isEqual(root);
        }

        if (current->isEqual(root)) {
          redraw = false;
        }
      }

      // Handle Up press
      if (btns.up) {
        if (current->getAbove()) {
          setCurrent(current->getAbove());
          redraw = true;
        }
      }

      if (btns.down) {
        if (current->getBelow()) {
          setCurrent(current->getBelow());
          redraw = true;
        }
      }

      if (redraw) {
        redrawMenu();
      }

      return(!current->isEqual(root));
    }

  private:
    void findFirst(MenuItem *newcur) {
      if (newcur) {
        if (newcur->getAbove()) {
          findFirst(newcur->getAbove());
        } else {
          first = newcur;
        }
      }
    }

    void redrawMenu() {
#if DEBUG
      DEBUGOUT.println(F("Redrawing Menu Display."));
#endif

      Adafruit_SSD1306 display = *oled;
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);

      MenuItem *cur = first;

      while (cur) {
        if (cur == current) {
          display.setTextColor(BLACK, WHITE);
        } else {
          display.setTextColor(WHITE);
        }

        display.println(cur->getLabel());
        cur = cur->getBelow();
      }

      display.display();
    }

    void setCurrent(MenuItem *newcur) {
      if (newcur) {
        current = newcur;
      }
    }

    Adafruit_SSD1306 *oled;
    MenuItem *root;
    MenuItem *current;
    MenuItem *first;
};

MenuItem mi_root("Main Menu", 1);

MenuItem mi_sensors("Sensors", 10);
MenuItem mi_sensor1("Sensor 1", 11);
MenuItem mi_sensor2("Sensor 2", 12);
MenuItem mi_sensor3("Sensor 3", 13);

MenuItem mi_master("Master On / Off", 20);
MenuItem mi_masteron("On", 21);
MenuItem mi_masteroff("Off", 22);

MenuDisplay menu(&display, &mi_root);

void setupRadio(void);
void setupDisplay(void);
void setupMenu(void);
void checkButtons(void);
void checkMenuDisplay(void);

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

  setupRadio();
  setupMenu();

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
  checkMenuDisplay();

  // Check Registered Sensors / Update Status
  // Check Servo Moving Status
  // Check Water Level Status
  // Check Spray Start / Stop Status
  // Check Radio For Packets
  // Update EEPROM Sensor Storage
  // Update Status Display

#if DEBUG
  // DEBUGOUT.println(F("LOOP"));
  // delay(500);
#endif
}

/*
 * Check Button States
 */
void checkButtons(void) {

  btnState.enter = 0;
  btnState.up = 0;
  btnState.down = 0;
  btnState.pushed = false;

  // Check if enter button pressed
  buttonEnter.update();
  if (buttonEnter.risingEdge()) {
    menuTimer = 0;
  } else if (buttonEnter.fallingEdge()) {
    btnState.pushed = true;
    btnState.enter = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
  }

  // Check if direction buttons pressed (if menu is active)
  if (!btnState.pushed && menuActive) {
    buttonUp.update();
    if (buttonUp.risingEdge()) {
      menuTimer = 0;
    } else if (buttonUp.fallingEdge()) {
      btnState.pushed = true;
      btnState.up = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
    }
  }

  if (!btnState.pushed && menuActive) {
    buttonDown.update();
    if (buttonDown.risingEdge()) {
      menuTimer = 0;
    } else if (buttonDown.fallingEdge()) {
      btnState.pushed = true;
      btnState.down = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
    }
  }

#if DEBUG
  if (btnState.pushed) {
    printf_P(
      PSTR("Button state read E: %d U: %d D: %d\r\n"),
      btnState.enter,
      btnState.up,
      btnState.down
    );
  }
#endif

}

/*
 * Check if displayed menu has update / timeout.
 */
void checkMenuDisplay(void) {
  if (menuActive && (menuTimer > MENU_TIMEOUT)) {
    menuActive = false;
    return;
  }

  // Show Menu if not active
  if ((!menuActive) && (btnState.enter > 0)) {
    menuActive = true;
    menuTimer = 0;
  }

  if (menuActive && btnState.pushed) {
    menuActive = menu.handleButtons(btnState);

#if DEBUG
    MenuItem cur = menu.getCurrent();
    printf_P(
      PSTR("checkMenuDisplay: Menu %s active, current now %s (%d).\r\n"),
      (menuActive) ? "is" : "is not",
      cur.getLabel(),
      cur.getUid()
    );
#endif

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
 * Setup Menu System
 */
void setupMenu(void) {

#if DEBUG
  DEBUGOUT.println(F("Initializing Menu:"));
#endif

  mi_root.setChild(mi_sensors);

  mi_sensor1.addBelow(mi_sensor2);
  mi_sensor2.addBelow(mi_sensor3);
  mi_sensor2.setParent(mi_sensors);
  mi_sensor3.setParent(mi_sensors);
  mi_sensors.setChild(mi_sensor1);

  mi_masteron.addBelow(mi_masteroff);
  mi_master.setChild(mi_masteron);
  mi_masteroff.setParent(mi_master);
  mi_master.setParent(mi_root);

  mi_sensors.addBelow(mi_master);
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
