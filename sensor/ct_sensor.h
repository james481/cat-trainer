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
#define RADIO_TIMEOUT 250

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
};

class Sensor {
  public:
    SensorUid id;
    uint8_t baseId = 0;
    uint8_t direction = 90;
    uint8_t lastSeen = 0;
    bool active = false;
    float vbat;

    bool isEqual(SensorUid &comp) {
      return(id.isEqual(comp));
    }

    bool isEqual(Sensor &comp) {
      return(id.isEqual(comp.id));
    }
};

/*
 * Data Packets
 */
struct DataPacket {
  SensorUid id;
  uint8_t ptype;
  uint8_t spipe;
  float batlevel;
};

const uint64_t control_pipes[2] = { 0xA0A0A0A0E1LL, 0xF0F0F0F0A1LL };
const uint64_t sensor_send_pipe_mask = 0xF0F0F0F000LL;
const uint64_t sensor_recv_pipe_mask = 0xD0D0D0D000LL;

// vim:cin:ai:sts=2 sw=2 ft=cpp
