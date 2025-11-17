#include <SPI.h>
#include <mcp2515.h>
 
struct can_frame canMsg;
MCP2515* mcp2515 = nullptr;  // Fixed: Initialize in setup()
int cntr = 0;
unsigned long oldTime = 0;
 
 
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

  Serial.println("------- CAN Speedtest ----------");
}

void loop() {
  if (mcp2515->readMessage(&canMsg) == MCP2515::ERROR_OK) {
    cntr++;
  }
 
  if ((millis()-oldTime)>1000) {
    oldTime = millis();
    Serial.print(cntr);
    Serial.println(" msg/sec");
    cntr = 0;      
  }
}
