#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
// NOTE: Use pointer and initialize in setup() to avoid ESP32 global initialization crash
// Global objects create FreeRTOS mutexes before the scheduler is ready
MCP2515* mcp2515 = nullptr;


void setup() {
  Serial.begin(115200);

  // Initialize MCP2515 after FreeRTOS is ready
  mcp2515 = new MCP2515(10);
  if (!mcp2515) {
    Serial.println("Failed to allocate MCP2515!");
    while(1) delay(1000);
  }

  mcp2515->reset();
  mcp2515->setBitrate(CAN_125KBPS);
  mcp2515->setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515->readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" ");
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");

    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}
