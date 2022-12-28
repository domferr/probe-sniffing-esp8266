#include <Arduino.h>

extern "C" {
  #include <user_interface.h>
}

#define DATA_LENGTH           112

#define FRAME_CONTROL_TYPE_MANAGEMENT 0
#define FRAME_CONTROL_SUBTYPE_PROBE_REQUEST 4

// https://espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf
struct RxControl {
  signed rssi:8;      // signal intensity of packet 
  unsigned rate:4; 
  unsigned is_group:1; 
  unsigned:1; 
  unsigned sig_mode:2;    // 0:is not 11n packet; non-0:is 11n packet; 
  unsigned legacy_length:12;  // if not 11n packet, shows length of packet. 
  unsigned damatch0:1; 
  unsigned damatch1:1; 
  unsigned bssidmatch0:1; 
  unsigned bssidmatch1:1; 
  unsigned MCS:7;     // if is 11n packet, shows the modulation and code used (range from 0 to 76)
  unsigned CWB:1;     // if is 11n packet, shows if is HT40 packet or not 
  unsigned HT_length:16;    // if is 11n packet, shows length of packet. 
  unsigned Smoothing:1; 
  unsigned Not_Sounding:1; 
  unsigned:1; 
  unsigned Aggregation:1; 
  unsigned STBC:2; 
  unsigned FEC_CODING:1;    // if is 11n packet, shows if is LDPC packet or not. 
  unsigned SGI:1; 
  unsigned rxend_state:8; 
  unsigned ampdu_cnt:8; 
  unsigned channel:4;   //which channel this packet in. 
  unsigned:12; 
};

struct SnifferPacket {
    struct RxControl rx_ctrl;
    uint8_t data[DATA_LENGTH];
    uint16_t cnt;
    uint16_t len;
};

struct frame_control {
  uint8_t protocol_version:2;
  uint8_t type:2;
  uint8_t subtype:4;

  uint8_t to_ds:1;
  uint8_t from_ds:1;
  uint8_t more_frag:1;
  uint8_t retry:1;
  uint8_t pwr_mgt:1;
  uint8_t more_data:1;
  uint8_t protected_frame:1;
  uint8_t order:1;
};

struct probe_request {
  struct frame_control frame_control;
  uint16_t duration;
  uint8_t destination[6];
  uint8_t source[6];
  uint8_t bssid[6];
  uint16_t number;
  uint8_t data[];
};

struct probe_tag {
  uint8_t tag_number;
  uint8_t tag_length;
  uint8_t data[];
};

static void printProbeRequest(struct probe_request *probeReq, int8_t rssi) {
  char addr[] = "00:00:00:00:00:00";
  sprintf(addr, "%02x:%02x:%02x:%02x:%02x:%02x", 
    probeReq->source[0], probeReq->source[1], probeReq->source[2], probeReq->source[3], probeReq->source[4], probeReq->source[5]);
  
  char ssid[33];
  struct probe_tag *tag = (struct probe_tag*) probeReq->data;

  if (tag->tag_number == 0 && tag->tag_length > 0 && tag->tag_length <= 32) {
    memcpy(ssid, tag->data, tag->tag_length);
    ssid[tag->tag_length] = 0;
  } else {
    ssid[0] = 0;
  }
  
  Serial.printf("RSSI: %d Ch: %d Source MAC: %s SSID: %s\n", 
                  rssi, 
                  wifi_get_channel(), 
                  addr,
                  ssid);
}

/** Callback for promiscuous mode */
static void ICACHE_FLASH_ATTR sniffer_callback(uint8_t *buffer, uint16_t length) {
  if (length == sizeof(struct SnifferPacket)) {
    struct SnifferPacket *snifferPacket = (struct SnifferPacket*) buffer;
    struct probe_request *probeReq = (struct probe_request*) snifferPacket->data;
    struct frame_control frameControl = probeReq->frame_control;
    
    // Only look for probe request packets
    if (frameControl.type != FRAME_CONTROL_TYPE_MANAGEMENT ||
        frameControl.subtype != FRAME_CONTROL_SUBTYPE_PROBE_REQUEST)
          return;
    
    printProbeRequest(probeReq, snifferPacket->rx_ctrl.rssi);
  }
}

#define CHANNEL_HOP_INTERVAL_MS   1000
static os_timer_t channelHop_timer;

/** Callback for channel hoping */
void channelHop() {
  // hoping channels 1-13
  uint8 new_channel = wifi_get_channel() + 1;
  if (new_channel > 13) {
    new_channel = 1;
  }
  wifi_set_channel(new_channel);
}

void setup() {
  // set the WiFi chip to "promiscuous" mode aka monitor mode
  Serial.begin(115200);
  delay(10);
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(1);
  wifi_promiscuous_enable(0);
  delay(10);
  wifi_set_promiscuous_rx_cb(sniffer_callback);
  delay(10);
  wifi_promiscuous_enable(1);

  // setup the channel hoping callback timer
  os_timer_disarm(&channelHop_timer);
  os_timer_setfn(&channelHop_timer, (os_timer_func_t *) channelHop, NULL);
  os_timer_arm(&channelHop_timer, CHANNEL_HOP_INTERVAL_MS, 1);
}

void loop() {
  delay(10);
}
