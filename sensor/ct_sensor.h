/*
 * ct_sensor.h
 *
 * Common definitions for trainer sensor communications.
 */

/*
 * Radio Packet IDs
 */
#define PTYPE_SYNCDATA 1
#define PTYPE_SYNCRES 2
#define PTYPE_SENSORDATA 3
#define PTYPE_STATUS 4
#define PTYPE_OK 5
#define PTYPE_DESYNC 6

/*
 * NRF24L01
 */
#define RADIO_CHANNEL 76
#define RADIO_TIMEOUT 500

/*
 * ID Storage (EEPROM)
 */
class SensorUid {
  public:
    char stype[11];
    uint8_t uid;

    bool isEqual(SensorUid &comp) {
      return((uid == comp.uid) && (strcmp(stype, comp.stype) == 0));
    }

    void copyFrom(SensorUid &source) {
      uid = source.uid;
      strncpy(stype, source.stype, sizeof(stype));
    }
};

class Sensor {
  public:
    SensorUid id;
    uint8_t baseId = 0;
    uint8_t direction = 90;
    uint8_t lastSeen = 0;
    bool active = false;
    uint8_t vbat = 0;
    uint16_t activated = 0;

    bool isEqual(SensorUid &comp) {
      return(id.isEqual(comp));
    }

    bool isEqual(Sensor &comp) {
      return(id.isEqual(comp.id));
    }

    void copyFrom(Sensor &source) {
      id.copyFrom(source.id);
      baseId = source.baseId;
      direction = source.direction;
      active = source.active;
      activated = source.activated;
    }
};

/*
 * Data Packets
 */
struct DataPacket {
  SensorUid id;
  uint8_t ptype;
  uint8_t spipe;
  uint8_t batlevel;
};

const uint64_t control_pipes[2] = { 0xF0F0F0F0AALL, 0xF0F0F0F0A1LL };
const uint64_t sensor_send_pipe_mask = 0xF0F0F0F000LL;

// vim:cin:ai:sts=2 sw=2 ft=cpp
