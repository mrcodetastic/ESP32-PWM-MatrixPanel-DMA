
#include <ESPAsyncE131.h>


uint8_t wsRawData[78 * 78];
class E131 {
private:
    // With 512 channels per universe, you need 24 universes to send all the data
    const uint8_t NUM_UNIVERSES = 12;
    const uint16_t CHANNELS_PER_UNIVERSE = 512;
    ESPAsyncE131 _e131;
    uint8_t colorToSet[3];

    void onNewPacketReceived(void* packet, protocol_t protocol, void* userInfo) {
        if (protocol == PROTOCOL_E131) {
            e131_packet_t* e131Packet = reinterpret_cast<e131_packet_t*>(packet);
            uint16_t offset = (htons(e131Packet->universe) - 1) * CHANNELS_PER_UNIVERSE;
            memcpy(wsRawData+offset, e131Packet->property_values + 1, CHANNELS_PER_UNIVERSE);  // +1 to skip the start code
//            lastPacketReceived = millis();
        }
    }
  

    
public:
    E131() {
        // Register the member function callback
        _e131.registerCallback([this](void* packet, protocol_t protocol, void* userInfo) {
            this->onNewPacketReceived(packet, protocol, userInfo);
        });
    }

    void begin() {                         // Listen via Unicast
      if (_e131.begin(E131_MULTICAST, 1, NUM_UNIVERSES, PROTOCOL_E131)) {
            Serial.println(F("Listening for data..."));
        } else {
            Serial.println(F("*** e131.begin failed ***"));
        }
    }
  
    // Call from the main loop depending on the frame-rate you want to achieve.
//    void show() {
//    int k = 0;
//    for (int i = posY; i < sizeY + posY; i++) {
//      for (int j = posX; j < sizeX + posX; j++) {
//        if (sprite[k] != 0 || sprite[k + 1] != 0 || sprite[k + 2] != 0) {
//          virtualDisp->drawPixelRGB888(j, i, sprite[k], sprite[k + 1], sprite[k + 2]);
//        }
//        k += 3;
//      }
//    }
//    // only needed for double-buffer
//    virtualDisp->flipDMABuffer();
//    }
};
