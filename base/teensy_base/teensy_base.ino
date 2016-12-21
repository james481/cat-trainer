/*
 * teensy_base.ino
 *
 * Behavior correction base station utilizing DC pump sprayhead and
 * communicating via NRF24L01+ to remote sensor nodes on an Teensy 3.x
 * compatible host.
 */

#include <functional>
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

#define MENU_TIMEOUT 10000 // ms until menu exits
#define SENSOR_MAX 4 // No more than 4 on NRF24L01
#define DISPLAY_INVERT 30 // Seconds between status display invert
#define DISPLAY_DIM 30 // Seconds until display dim
#define WATER_LEVEL_SENSOR false
#define EEPROM_SENSORS_ADDR 0
#define DEBUG true
#define DEBUGOUT Serial

// NRF24L01
#define GPIO_RF24_CE 14
#define GPIO_RF24_CSN 10

// Buttons
#define GPIO_BTN_ENTER 6
#define GPIO_BTN_DN 7
#define GPIO_BTN_UP 8
#define GPIO_BTN_DEBOUNCE 10
#define GPIO_BTN_HOLD_DELAY 500

// Pump / Servo
#define SERVO_MOVE_DELAY 500 // ms for servo to repoint nozzle
#define PUMP_SPRAY_LENGTH 500 // ms for spray pump active
#define GPIO_SPRAY_PUMP A6
#define GPIO_SPRAY_PUMP2 A7
#define GPIO_SPRAY_SERVO 3
#define SERVO_MIN_LEN 1050
#define SERVO_MAX_LEN 2050

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
Bounce buttonEnter(GPIO_BTN_ENTER, GPIO_BTN_DEBOUNCE);
Bounce buttonDown(GPIO_BTN_DN, GPIO_BTN_DEBOUNCE);
Bounce buttonUp(GPIO_BTN_UP, GPIO_BTN_DEBOUNCE);

// Button States: 0 - None, 1 - Click, 2 - Hold
struct ButtonState {
  uint8_t enter = 0;
  uint8_t up = 0;
  uint8_t down = 0;
  bool pushed = false;
} btnState;

/*
 * Sensor ID / Packet Storage
 */
Sensor sensors[SENSOR_MAX];
SensorUid remoteUid;
DataPacket recvPacket, sendPacket;

const uint8_t sensorBaseIds[4] = { 0xB1, 0xC2, 0xD3, 0xE4 };

/*
 * Master / Spray / Movement / Menu States and Timers
 */
bool masterActive = true;
bool menuActive = false;
bool redrawStatus = false;
int8_t sprayActive = -1;
int8_t sprayNeeded = -1;

uint8_t sprayPos = 90;
uint8_t sprayMoving = 0;

bool displayInvert = false;
uint8_t displayInvertTimer = 0;

bool displayDim = false;
uint8_t displayDimTimer = 0;

elapsedMillis sprayTimer;
elapsedMillis statusTimer;
elapsedMillis menuTimer;

/*
 * Menu Classes
 */
class MenuItem;
class MenuDisplay;

class MenuItem {
  public:
    MenuItem(const char* label, const uint8_t uid) : itemLabel(label), itemUid(uid) {
      parent = child = above = below = 0;
    }

    const bool isActive() {
      if (activeCb) {
        return(activeCb(*this));
      }
      return(true);
    }

    const char* getLabel(bool useCb = true) {
      if (labelCb && useCb) {
        return(labelCb(*this));
      }
      return(itemLabel);
    }

    char* getValue() {
      if (valueCb) {
        valueCb(*this, itemVal);
      }

      return(itemVal);
    }

    const uint8_t getUid() const {
      return(itemUid);
    }

    MenuItem *getAbove(bool onlyActive = false) const {
      if (onlyActive && above) {
        return((above->isActive()) ? above : above->getAbove(true));
      }
      return(above);
    }

    MenuItem *getBelow(bool onlyActive = false) {
      if (onlyActive && below) {
        return((below->isActive()) ? below : below->getBelow(true));
      }
      return(below);
    }

    MenuItem *getChild(bool onlyActive = true, bool useCb = true) {
      if (childCb && useCb) {
        return(childCb(*this));
      }

      if (onlyActive && child && !child->isActive() && child->getBelow()) {
        return(child->getBelow(true));
      }

      return(child);
    }

    MenuItem *getParent() const {
      return(parent);
    }

    bool getItemEditable() {
      return(itemEditable);
    }

    void handleEdit(ButtonState btns) {
      if (editCb) {
        editCb(*this, btns);
      }
    }

    MenuItem &setParent(MenuItem &item) {
      parent = &item;
      return(*this);
    }

    MenuItem &setChild(MenuItem &item) {
      child = &item;
      item.parent = this;

      return(*this);
    }

    MenuItem &setItemEditable(bool editable) {
      itemEditable = editable;
      return(*this);
    }

    MenuItem &addAbove(MenuItem &item) {
      item.below = this;
      above = &item;
      return(*this);
    }

    MenuItem &addBelow(MenuItem &item) {
      item.above = this;
      below = &item;
      return(*this);
    }

    bool hasValue() {
      return((valueCb) ? true : false);
    }

    bool isEqual(MenuItem &item) {
      return(getUid() == item.getUid());
    }

    bool isEqual(MenuItem *item) {
      return(getUid() == item->getUid());
    }

    MenuItem &setActiveCb(std::function<bool (MenuItem &item)> acb) {
      activeCb = acb;
      return(*this);
    }

    MenuItem &setChildCb(std::function<MenuItem* (MenuItem &item)> cb) {
      childCb = cb;
      return(*this);
    }

    MenuItem &setEditCb(std::function<void (MenuItem &item, ButtonState &btns)> cb) {
      editCb = cb;
      return(*this);
    }

    MenuItem &setLabelCb(std::function<const char* (MenuItem &item)> cb) {
      labelCb = cb;
      return(*this);
    }

    MenuItem &setValueCb(std::function<void (MenuItem &item, char* val)> cb) {
      valueCb = cb;
      return(*this);
    }

  protected:
    const char* itemLabel;
    char itemVal[10];
    const uint8_t itemUid;
    bool itemEditable = false;

    // Linked Matrix
    MenuItem *above;
    MenuItem *below;
    MenuItem *child;
    MenuItem *parent;

    // Event Callback Lambdas
    std::function<bool (MenuItem &item)> activeCb;
    std::function<MenuItem* (MenuItem &item)> childCb;
    std::function<void (MenuItem &item, ButtonState &btns)> editCb;
    std::function<const char* (MenuItem &item)> labelCb;
    std::function<void (MenuItem &item, char* val)> valueCb;
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

    bool handleButtons(ButtonState btns) {
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
        if (isEditing) {
          isEditing = false;
          redraw = true;
        } else if (current->getItemEditable()) {
          isEditing = true;
          redraw = true;
        } else if (current->getChild()) {
          setCurrent(current->getChild());
          findFirst(current);
          redraw = true;
        }
      } else if (btns.enter == 2) {
        if (isEditing) {
          isEditing = false;
          redraw = true;
        } else if (current->getParent()) {
          setCurrent(current->getParent());
          findFirst(current);
          redraw = !current->isEqual(root);
        }

        if (current->isEqual(root)) {
          redraw = false;
        }
      }

      // Handle Up / Down press
      if (isEditing && (btns.up || btns.down)) {
        current->handleEdit(btns);
        redraw = true;
      } else if (btns.up && current->getAbove(true)) {
        setCurrent(current->getAbove(true));
        redraw = true;
      } else if (btns.down && current->getBelow(true)) {
        setCurrent(current->getBelow(true));
        redraw = true;
      }

      if (redraw) {
        redrawMenu();
      }

      return(!current->isEqual(root));
    }

    void resetMenu(void) {
      isEditing = false;
      setCurrent(root);
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
      display.dim(false);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.invertDisplay(0);

      MenuItem *cur = first;

      if (cur->getParent()) {
        display.println(cur->getParent()->getLabel());
        display.println();
      }

      while (cur) {
        if (cur == current) {
          display.setTextColor(BLACK, WHITE);
        } else {
          display.setTextColor(WHITE);
        }

        if (cur->isActive()) {
          if (cur->hasValue()) {

            int padding =
              21 - strlen(cur->getLabel()) - strlen(cur->getValue());
            padding = (padding > 0) ? padding : 0;

            if (isEditing) { display.setTextColor(WHITE); }

            display.print(cur->getLabel());

            while (padding > 0) {
              display.print(" ");
              padding--;
            }

            if ((cur == current) && isEditing) {
              display.setTextColor(BLACK, WHITE);
            }

            display.println(cur->getValue());
          } else {
            display.println(cur->getLabel());
          }
        }

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
    bool isEditing = false;
};

// Menu item objects, note UIDs are used for functional purposes.
MenuItem mi_root("Main Menu", 1);

MenuItem mi_sensors("Sensors", 9);
MenuItem mi_sensor1("Sensor 1", 10);
MenuItem mi_sensor2("Sensor 2", 11);
MenuItem mi_sensor3("Sensor 3", 12);
MenuItem mi_sensor4("Sensor 4", 13);
MenuItem mi_sensorN("No Sensors Found", 15);

MenuItem mi_sensor_active("Active:", 20);
MenuItem mi_sensor_dir("Direction:", 21);
MenuItem mi_sensor_vbat("Battery:", 22);
MenuItem mi_sensor_lastseen("Last Seen:", 23);
MenuItem mi_sensor_forget("Remove:", 24);

MenuItem mi_master("Master Active:", 100);
MenuItem mi_sensors_save("Save Setup:", 101);
MenuItem mi_confirm_n("Cancel", 110);
MenuItem mi_confirm_y("Confirm", 111);

MenuDisplay menu(&display, &mi_root);

/*
 * This ugly bit of hackery is needed for functional
 * to compile using some types of lambdas.
 */
namespace std
{
  void __throw_bad_function_call()
  {
  }
}

void setup(void);
void loop(void);
void setupRadio(void);
void setupDisplay(void);
void setupMenu(void);
void checkButtons(void);
void checkMenuDisplay(void);
void checkSprayServoTimeout(void);
void checkStartSpray(void);
void handleRadioPacket(void);
void handleSensorData(uint8_t sindex, DataPacket &status);
void handleSensorStatus(uint8_t sindex, DataPacket &status);
bool handleSyncRequest(DataPacket &req);
bool acknowledgeSensor(Sensor &sen);
bool desyncronizeSensor(SensorUid &sid);
void redrawStatusDisplay(void);
void saveSensorSetup(void);
void loadSensorSetup(void);

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
  pinMode(GPIO_BTN_ENTER, INPUT_PULLUP);
  pinMode(GPIO_BTN_DN, INPUT_PULLUP);
  pinMode(GPIO_BTN_UP, INPUT_PULLUP);

  // Setup Spray Pump
  pinMode(GPIO_SPRAY_PUMP, OUTPUT);
  analogWrite(GPIO_SPRAY_PUMP, 0);
  pinMode(GPIO_SPRAY_PUMP2, OUTPUT);
  analogWrite(GPIO_SPRAY_PUMP2, 0);

  // Setup Servo
  sprayServo.attach(GPIO_SPRAY_SERVO, SERVO_MIN_LEN, SERVO_MAX_LEN);
  sprayServo.write(90);
  sprayPos = 90;

  setupRadio();
  setupMenu();

  // Load Saved Sensors From EEPROM
  loadSensorSetup();

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

  // Check Spray / Servo Moving Timeouts.
  checkSprayServoTimeout();

  // Check Input Button Status
  checkButtons();

  // Check Menu / Timeout Status
  checkMenuDisplay();

  // Check Spray Start / Stop Status
  checkStartSpray();

  // Check Radio For Packets
  handleRadioPacket();

  // TODO Update EEPROM Sensor Storage

  // Update Status / Display
  if (statusTimer > 1000) {

    // Check Registered Sensors / Update Status
    for (int i = 0; i < SENSOR_MAX; i++) {
      if (sensors[i].lastSeen < 255) {
        sensors[i].lastSeen++;
      }
    }

    // TODO Check Water Level Status
#if WATER_LEVEL_SENSOR
#endif

    // Update Status Display
    redrawStatus = true;
    if (displayInvertTimer > DISPLAY_INVERT) {
      displayInvert = !displayInvert;
      displayInvertTimer = 0;
    }
    displayInvertTimer++;

    if (displayDimTimer > DISPLAY_DIM) {
      displayDim = true;
    } else {
      displayDim = false;
      displayDimTimer++;
    }

    statusTimer = 0;
  }

  if (!menuActive && redrawStatus) {
    redrawStatusDisplay();
  }
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
  if (buttonEnter.fallingEdge()) {
    menuTimer = 0;
  } else if (buttonEnter.risingEdge()) {
    btnState.pushed = true;
    btnState.enter = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
  }

  // Check if direction buttons pressed (if menu is active)
  if (!btnState.pushed && menuActive) {
    buttonUp.update();
    if (buttonUp.fallingEdge()) {
      menuTimer = 0;
    } else if (buttonUp.risingEdge()) {
      btnState.pushed = true;
      btnState.up = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
    }
  }

  if (!btnState.pushed && menuActive) {
    buttonDown.update();
    if (buttonDown.fallingEdge()) {
      menuTimer = 0;
    } else if (buttonDown.risingEdge()) {
      btnState.pushed = true;
      btnState.down = (menuTimer > GPIO_BTN_HOLD_DELAY) ? 2 : 1;
    }
  }

#if DEBUG
  if (btnState.pushed) {
    printf_P(
      PSTR("checkButtons: Button state read E: %d U: %d D: %d\r\n"),
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
  // Timeout Menu display
  if (menuActive && (menuTimer > MENU_TIMEOUT)) {
    menuActive = false;
    menuTimer = 0;
    menu.resetMenu();
    return;
  }

  // Show Menu if not active
  if ((!menuActive) && (btnState.enter > 0)) {
    menuActive = true;
    menuTimer = 0;
  }

  if (menuActive && btnState.pushed) {
    displayDim = false;
    displayDimTimer = 0;
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
 * Check Timeouts of Spray / Servo Moving.
 */
void checkSprayServoTimeout(void) {
  // Check Spray timeout status
  if ((sprayActive != -1) && (sprayTimer > PUMP_SPRAY_LENGTH)) {
    analogWrite(GPIO_SPRAY_PUMP2, 0);
    sprayActive = -1;
    sprayTimer = 0;
  }

  // Check Servo Moving Status
  if ((sprayMoving > 0) && (sprayTimer > SERVO_MOVE_DELAY)) {
    sprayPos = sprayMoving;
    sprayMoving = 0;
    sprayTimer = 0;
  }
}

/*
 * Check Spray Needed Status And Start Spray / Servo Move.
 */
void checkStartSpray(void) {
  // Check Spray Start / Stop Status
  if (
    masterActive &&
    !menuActive &&
    (sprayNeeded != -1) &&
    (sprayMoving == 0) &&
    (sprayActive == -1)
  ) {

    Sensor sens = sensors[sprayNeeded];
    if (!sens.baseId || !sens.active) {

#if DEBUG
      printf_P(
        PSTR("checkStartSpray: Sensor not active or no baseId for %d.\r\n"),
        sprayNeeded
      );
#endif

      sprayNeeded = -1;
      return;
    }

    // Check servo position
    if (sens.direction == sprayPos) {
      // Activate Spray
      sprayActive = sprayNeeded;
      sprayNeeded = -1;
      analogWrite(GPIO_SPRAY_PUMP2, 255);

#if DEBUG
      printf_P(
        PSTR("checkStartSpray: Activating spray towards sensor %s (%d).\r\n"),
        sens.id.stype,
        sens.id.uid
      );
#endif

    } else {
      // Move servo to spray position.
      sprayMoving = sens.direction;
      sprayServo.write(sens.direction);

#if DEBUG
      printf_P(
        PSTR("checkStartSpray: Moving servo towards sensor %s (%d): %d.\r\n"),
        sens.id.stype,
        sens.id.uid,
        sens.direction
      );
#endif

    }

    sprayTimer = 0;
  }
}

/*
 * Acknowledge Sensor Or Status Data With OK
 */
bool acknowledgeSensor(Sensor &sen) {
  bool sent = false;
  radio.stopListening();

  sendPacket.id.copyFrom(sen.id);
  sendPacket.ptype = PTYPE_OK;

  radio.openWritingPipe(sensor_send_pipe_mask | sen.baseId);
  sent = radio.write(&sendPacket, sizeof(DataPacket));

#if DEBUG
  if (sent) {
    printf_P(
      PSTR("acknowledgeSensor: Sent Acknowledge to %s (%d) on pipe %02X.\r\n"),
      sen.id.stype,
      sen.id.uid,
      sen.baseId
    );
  } else {
    printf_P(
      PSTR("acknowledgeSensor: Failed to send Acknowledge to %s (%d) on pipe %02X.\r\n"),
      sen.id.stype,
      sen.id.uid,
      sen.baseId
    );
  }
#endif

  radio.startListening();
  return(sent);
}

/*
 * Send A Desyncronize Packet To A Sensor By ID
 */
bool desyncronizeSensor(SensorUid &sid) {
  bool sent = false;
  radio.stopListening();

  sendPacket.id.copyFrom(sid);
  sendPacket.ptype = PTYPE_DESYNC;

  radio.openWritingPipe(control_pipes[0]);
  sent = radio.write(&sendPacket, sizeof(DataPacket));

#if DEBUG
  if (sent) {
    printf_P(
      PSTR("desyncronizeSensor: Sent Desyncronize request to %s (%d).\r\n"),
      sid.stype,
      sid.uid
    );
  } else {
    printf_P(
      PSTR("desyncronizeSensor: Failed to send Desyncronize request to %s (%d).\r\n"),
      sid.stype,
      sid.uid
    );
  }
#endif

  radio.startListening();
  return(sent);
}

/*
 * Handle Sensor Trigger Data Packet
 */
void handleSensorData(uint8_t sindex, DataPacket &status) {
  if (sensors[sindex].baseId) {
    sensors[sindex].lastSeen = 0;
    sensors[sindex].vbat = status.batlevel;

    // TODO Debounce / Queue Triggers
    if (masterActive && sensors[sindex].active && (sprayNeeded == -1)) {
      if (sensors[sindex].activated < 999) {
        sensors[sindex].activated++;
      }

      sprayNeeded = sindex;
    }

#if DEBUG
    printf_P(
      PSTR("handleSensorData: Handle sensor trigger %s (%d): %d.\r\n"),
      sensors[sindex].id.stype,
      sensors[sindex].id.uid,
      sensors[sindex].vbat
    );
#endif
  }
}

/*
 * Handle Sensor Status Update Packet
 */
void handleSensorStatus(uint8_t sindex, DataPacket &status) {
  // Reset sensor last seen / battery voltage
  if (sensors[sindex].baseId) {
    sensors[sindex].lastSeen = 0;
    sensors[sindex].vbat = status.batlevel;

#if DEBUG
    printf_P(
      PSTR("handleSensorStatus: Updated sensor status %s (%d): %d.\r\n"),
      sensors[sindex].id.stype,
      sensors[sindex].id.uid,
      sensors[sindex].vbat
    );
#endif
  }
}

/*
 * Handle Request To Syncronize Sensor
 */
bool handleSyncRequest(DataPacket &req) {
  bool sent = false;
  int sindex = -1;
  int freeindex = -1;
  uint8_t newId = 0;

  // Find existing or free sensor in storage
  for (int i = 0; i < SENSOR_MAX; i++) {
    if (sensors[i].isEqual(req.id)) {
      sindex = i;
      break;
    }

    if (!sensors[i].baseId && (freeindex == -1)) {
      freeindex = i;
    }
  }

  if (sindex != -1) {
    sensors[sindex].lastSeen = 0;
    sensors[sindex].vbat = req.batlevel;
    newId = sensors[sindex].baseId;

#if DEBUG
    printf_P(
      PSTR("handleSyncRequest: Found existing record %d for %s (%d): %02X.\r\n"),
      sindex,
      sensors[sindex].id.stype,
      sensors[sindex].id.uid,
      newId
    );
#endif

  } else if (freeindex != -1) {
    // Create sensor storage
    sensors[freeindex].id.copyFrom(req.id);
    sensors[freeindex].lastSeen = 0;
    sensors[freeindex].vbat = req.batlevel;
    newId = sensorBaseIds[freeindex];
    sensors[freeindex].baseId = newId;
    sensors[freeindex].active = true;

#if DEBUG
    printf_P(
      PSTR("handleSyncRequest: Stored new record %d for %s (%d): %02X.\r\n"),
      freeindex,
      sensors[freeindex].id.stype,
      sensors[freeindex].id.uid,
      newId
    );
#endif

  } else {
    newId = 0;

#if DEBUG
    printf_P(
      PSTR("handleSyncRequest: Unable to store record for %s (%d).\r\n"),
      req.id.stype,
      req.id.uid
    );
#endif

  }

  // Return response with pipe id.
  radio.stopListening();

  sendPacket.id.copyFrom(req.id);
  sendPacket.ptype = PTYPE_SYNCRES;
  sendPacket.spipe = newId;
  sendPacket.batlevel = req.batlevel;

  radio.openWritingPipe(control_pipes[0]);
  sent = radio.write(&sendPacket, sizeof(DataPacket));

#if DEBUG
  if (sent) {
    printf_P(
      PSTR("handleSyncRequest: Sent Syncronize response to %s (%d): %02X.\r\n"),
      sendPacket.id.stype,
      sendPacket.id.uid,
      sendPacket.spipe
    );
  } else {
    printf_P(
      PSTR("handleSyncRequest: Failed to send Syncronize response to %s (%d).\r\n"),
      sendPacket.id.stype,
      sendPacket.id.uid
    );
  }
#endif

  radio.startListening();
  return(sent);
}

/*
 * Check / Handle Incoming Radio Packets
 */
void handleRadioPacket(void) {
  uint8_t baseId, pipe_num, sindex = 0;
  bool desync = false;
  Sensor recvSensor;

  if (radio.available(&pipe_num)) {
    radio.read(&recvPacket, sizeof(DataPacket));

    if (!recvPacket.ptype || !recvPacket.id.uid) {
#if DEBUG
      DEBUGOUT.println(F("handleRadioPacket: Invalid Packet Received."));
#endif
      return;
    }

    if (pipe_num < 2) {
      // Sync request
      handleSyncRequest(recvPacket);
    } else {
      // Identify expected sensor baseId from pipe
      baseId = sensorBaseIds[pipe_num - 2];

      // Find registered sensor by baseId
      for (int i = 0; i < SENSOR_MAX; i++) {
        if (sensors[i].baseId == baseId) {
          recvSensor = sensors[i];
          sindex = i;
          break;
        }
      }

      // Check if sent packet uid matches sensors uid
      if (recvSensor.baseId && (recvSensor.isEqual(recvPacket.id))) {
        if (recvPacket.ptype == PTYPE_SENSORDATA) {
          handleSensorData(sindex, recvPacket);
        } else if (recvPacket.ptype == PTYPE_STATUS) {
          handleSensorStatus(sindex, recvPacket);
        } else {
          // Unknown Packet Type
          desync = true;
        }
      } else {
        // Unable to find sensor or id mismatch, desync.
#if DEBUG
        printf_P(
          PSTR("handleRadioPacket: Unable to find stored sensor %s (%d).\r\n"),
          recvPacket.id.stype,
          recvPacket.id.uid
        );
#endif
        desync = true;
      }

      // Send Ack or Desync packets
      if (desync) {
        desyncronizeSensor(recvPacket.id);
      } else {
        acknowledgeSensor(recvSensor);
      }
    }
  }
}

/*
 * Redraw the status / rest screen.
 */
void redrawStatusDisplay(void) {
  display.clearDisplay();
  display.dim(displayDim);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Show Sensor Status Indicator
  uint8_t pos = 0;
  for (int i = 0; i < SENSOR_MAX; i++) {
    if (sensors[i].baseId) {

      // Draw sensor activated count / battery status icon
      char activated[4];
      if (!sensors[i].active || !masterActive) {
        activated[0] = 'X';
        activated[1] = 0;
      } else if (sensors[i].lastSeen > 60) {
        activated[0] = '?';
        activated[1] = 0;
      } else {
        snprintf(activated, 4, "%d", sensors[i].activated);
      }

      char direction[4];
      snprintf(direction, 4, "%d", sensors[i].direction);

      // Print sensor activated count (top)
      display.setCursor(5 + (25 * pos) + ((3 - strlen(activated)) * 3), 2);
      display.print(activated);

      // Draw battery icon
      uint8_t batIconHeight = 38;
      display.drawRoundRect(9 + (25 * pos), 11, 10, 5, 2, WHITE);
      display.drawRoundRect(3 + (25 * pos), 14, 22, batIconHeight, 3, WHITE);
      display.fillRect(10 + (25 * pos), 15, 8, 2, BLACK);

      // Fill battery icon relative to bat level
      // LiIon bat: vbat 42 = 100%, vbat 32 = 0%
      uint8_t vbat = sensors[i].vbat;
      vbat = (vbat > 42) ? 42 : vbat;
      vbat = (vbat < 32) ? 32 : vbat;
      float batp = (vbat - 32) / 10.0;
      int batLevelHeight = (int) (batIconHeight * batp);
      display.fillRoundRect(
        3 + (25 * pos),
        14 + (batIconHeight - batLevelHeight),
        22,
        batLevelHeight,
        3,
        WHITE
      );

      // Print sensor servo direction (bottom)
      display.setCursor(5 + (25 * pos) + ((3 - strlen(direction)) * 3), 55);
      display.print(direction);
      pos++;
    }

    if (!pos) {
      display.setTextSize(2);
      display.setCursor(2, 10);
      display.println("   No");
      display.println(" Sensors");
    }
  }

#if WATER_LEVEL_SENSOR
  // Show Water Level Indicator
  display.drawRoundRect(110, 20, 24, 50, 2, WHITE);
#endif

  // Show Master Active Flag
  display.setTextSize(1);
  if (masterActive) {
    display.setCursor(115, 6);
    display.print("On");
  } else {
    display.setCursor(105, 6);
    display.print("Off");
  }

  display.invertDisplay(displayInvert);
  display.display();
}

/*
 * Setup Display / Show Splash Screen
 */
void setupDisplay(void) {

#if DEBUG
  DEBUGOUT.println(F("setupDisplay: Initializing Display."));
#endif

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("    Cat");
  display.println("  Trainer");
  display.println("");
  display.setTextSize(1);
  display.println("         v0.1");
  display.display();

  delay(3000);
}

/*
 * Setup Menu System
 */
void setupMenu(void) {

#if DEBUG
  DEBUGOUT.println(F("setupMenu: Initializing Menu."));
#endif

  // Hide sensor items that aren't synced
  std::function<bool (MenuItem &item)> sensActiveCb = [] (MenuItem &item) {
    bool sensActive = true;
    uint8_t sensorId = item.getUid() - 10;
    sensActive = (sensors[sensorId].baseId > 0) ? true : false;
    return(sensActive);
  };

  // Display "none" sensor item if no sensors found
  std::function<bool (MenuItem &item)> noneActiveCb = [] (MenuItem &item) {
    bool noneActive = true;
    for (int i = 0; i < SENSOR_MAX; i++) {
      if (sensors[i].baseId) {
        noneActive = false;
        break;
      }
    }

    return(noneActive);
  };

  // Render sensor items as sensor name
  std::function<const char* (MenuItem &item)> labelCb = [] (MenuItem &item) {
    uint8_t sensorId = item.getUid() - 10;
    if (sensors[sensorId].baseId) {
      const char* type = sensors[sensorId].id.stype;
      return(type);
    }

    return(item.getLabel(false));
  };

  // Add sensor edit items dynamically as sensor menu children
  std::function<MenuItem* (MenuItem &item)> sensChildCb = [] (MenuItem &item) {
    mi_sensor_active.setParent(item);
    mi_sensor_dir.setParent(item);
    mi_sensor_vbat.setParent(item);
    mi_sensor_lastseen.setParent(item);
    return(&mi_sensor_active);
  };

  mi_root.setChild(mi_sensors);
  mi_sensors.setChild(mi_sensor1);

  // Sensor Items
  mi_sensor1
    .setActiveCb(sensActiveCb)
    .setChildCb(sensChildCb)
    .setLabelCb(labelCb)
    .addBelow(mi_sensor2);

  mi_sensor2
    .setActiveCb(sensActiveCb)
    .setChildCb(sensChildCb)
    .setLabelCb(labelCb)
    .setParent(mi_sensors)
    .addBelow(mi_sensor3);

  mi_sensor3
    .setActiveCb(sensActiveCb)
    .setChildCb(sensChildCb)
    .setLabelCb(labelCb)
    .setParent(mi_sensors)
    .addBelow(mi_sensor4);

  mi_sensor4
    .setActiveCb(sensActiveCb)
    .setChildCb(sensChildCb)
    .setLabelCb(labelCb)
    .setParent(mi_sensors)
    .addBelow(mi_sensorN);

  mi_sensorN
    .setActiveCb(noneActiveCb)
    .setParent(mi_sensors);

  // Sensor Status / Edit Items
  mi_sensor_active
    .setParent(mi_sensor1)
    .addBelow(mi_sensor_dir)
    .setItemEditable(true)
    .setValueCb([] (MenuItem &item, char* val) {
      if (item.getParent()) {
        uint8_t sensorId = item.getParent()->getUid() - 10;
        if (sensors[sensorId].baseId) {
          const char* active = (sensors[sensorId].active) ? "On" : "Off";
          strncpy(val, active, 9);
        }
      }
    })
    .setEditCb([] (MenuItem &item, ButtonState &btns) {
      if (item.getParent()) {
        uint8_t sensorId = item.getParent()->getUid() - 10;
        if (sensors[sensorId].baseId) {
          if (btns.up || btns.down) {
            sensors[sensorId].active = !sensors[sensorId].active;
          }
        }
      }
    });

  mi_sensor_dir
    .setParent(mi_sensor1)
    .addBelow(mi_sensor_vbat)
    .setItemEditable(true)
    .setValueCb([] (MenuItem &item, char* val) {
      if (item.getParent()) {
        uint8_t sensorId = item.getParent()->getUid() - 10;
        if (sensors[sensorId].baseId) {
          snprintf(val, 9, "%d", sensors[sensorId].direction);
        }
      }
    })
    .setEditCb([] (MenuItem &item, ButtonState &btns) {
      if (item.getParent()) {
        uint8_t sensorId = item.getParent()->getUid() - 10;
        if (sensors[sensorId].baseId) {
          uint8_t dir = sensors[sensorId].direction;
          if (btns.up && (dir < 180)) {
            sensors[sensorId].direction += ((dir < 170) ? 10 : 2);
          } else if (btns.down && (dir > 1)) {
            sensors[sensorId].direction -= ((dir > 11) ? 10 : 2);
          }

          sprayServo.write(sensors[sensorId].direction);
        }
      }
    });

  mi_sensor_vbat
    .setParent(mi_sensor1)
    .addBelow(mi_sensor_lastseen)
    .setValueCb([] (MenuItem &item, char* val) {
      if (item.getParent()) {
        uint8_t sensorId = item.getParent()->getUid() - 10;
        if (sensors[sensorId].baseId) {
          uint8_t vwhole, vpart;
          vwhole = sensors[sensorId].vbat / 10;
          vpart = sensors[sensorId].vbat % 10;
          snprintf(val, 9, "%d.%d", vwhole, vpart);
        }
      }
    });

  mi_sensor_lastseen
    .setParent(mi_sensor1)
    .setValueCb([] (MenuItem &item, char* val) {
      if (item.getParent()) {
        uint8_t sensorId = item.getParent()->getUid() - 10;
        if (sensors[sensorId].baseId) {
          snprintf(val, 9, "%d", sensors[sensorId].lastSeen);
        }
      }
    });

  // Master Active Setting
  mi_master
    .setParent(mi_root)
    .setItemEditable(true)
    .setValueCb([] (MenuItem &item, char* val) {
      const char* active = (masterActive) ? "On" : "Off";
      strncpy(val, active, 9);
    })
    .setEditCb([] (MenuItem &item, ButtonState &btns) {
      if (btns.up || btns.down) {
        masterActive = !masterActive;
      }
    });

  mi_sensors.addBelow(mi_master);

  // Save Current Sensors

}

/*
 * Setup NRF24L01+ chip.
 */
void setupRadio(void) {

#if DEBUG
  DEBUGOUT.println(F("setupRadio: Initializing NRF24L01+:"));
#endif

  radio.begin();

  radio.setPayloadSize(sizeof(DataPacket));
  radio.setAutoAck(1);

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(RADIO_CHANNEL);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(15,15);

  radio.openWritingPipe(control_pipes[0]);
  radio.openReadingPipe(1,control_pipes[1]);

  for (int i = 2; i < SENSOR_MAX + 2; i++) {
    uint8_t pipeBaseId = sensorBaseIds[i - 2];
    radio.openReadingPipe(i, sensor_send_pipe_mask | pipeBaseId);
  }

  radio.startListening();

#if DEBUG
  radio.printDetails();
  delay(1000);
#endif
}

/*
 * Save Current Sensors Config to EEPROM.
 */
void saveSensorSetup(void) {
  Sensor store[SENSOR_MAX];

  uint8_t storei = 0;
  for (int i = 0; i < SENSOR_MAX; i++) {
    if (sensors[i].baseId) {
      store[storei].copyFrom(sensors[i]);
      storei++;
    }
  }

  if (storei > 0) {

#if DEBUG
    printf_P(
      PSTR("saveSensorSetup: Saving %d sensor records.\r\n"),
      storei
    );
#endif

    EEPROM.put(EEPROM_SENSORS_ADDR, store);
  } else {

#if DEBUG
  DEBUGOUT.println(F("saveSensorSetup: No sensor records found."));
#endif

  }
}

/*
 * Load Saved Sensors Config from EEPROM.
 */
void loadSensorSetup(void) {
  Sensor load[SENSOR_MAX];

  uint8_t loadi = 0;

  EEPROM.get(EEPROM_SENSORS_ADDR, load);

  for (int i = 0; i < SENSOR_MAX; i++) {
    if (load[i].baseId && load[i].id.uid) {
      sensors[loadi].copyFrom(load[i]);
      loadi++;
    }
  }

#if DEBUG
    printf_P(
      PSTR("loadSensorSetup: Loaded %d sensor records.\r\n"),
      loadi
    );
#endif

}

// vim:cin:ai:sts=2 sw=2 ft=cpp
