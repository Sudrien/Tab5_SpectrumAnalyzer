/*
  Serial heartbeat — proves the console works, nothing else.

  Flash this, open tio, and watch. If the counter climbs, the serial
  path is good and any missing output from another sketch is that
  sketch's problem, not the console. If you see nothing here, the
  problem is the port / tio / USB-CDC link and no sketch will show
  output until that is fixed.

  Build with the SAME flags as everything else in this project so the
  USB-CDC config matches:
    arduino-cli compile \
      --fqbn "esp32:esp32:esp32p4:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc" .
    arduino-cli upload  \
      --fqbn "esp32:esp32:esp32p4:FlashSize=16M,PSRAM=enabled,PartitionScheme=app3M_fat9M_16MB,CDCOnBoot=cdc" \
      --port /dev/ttyACM0 .

  Then:  tio -b 115200 /dev/ttyACM0
*/

#include <M5Unified.h>

uint32_t n = 0;

void setup() {
  Serial.begin(115200);
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(3);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN);
  M5.Display.setTextSize(4);
  M5.Display.drawString("Serial heartbeat", 20, 20);

  // A few immediate lines so there is something even if the 1 Hz
  // loop output is what is going missing.
  for (int i = 0; i < 5; i++) {
    Serial.printf("BOOT line %d — if you can read this, output works\n", i);
    delay(200);
  }
}

void loop() {
  M5.update();

  // Print on both Serial and, if it exists as a distinct object on this
  // core, the USB CDC object — whichever one tio is actually reading.
  Serial.printf("HEARTBEAT %lu\n", (unsigned long)n);

  // Mirror to the screen so you can confirm the sketch is alive even
  // with no console at all. If this number climbs on the display but
  // not in tio, the sketch is fine and the console link is the problem.
  M5.Display.fillRect(20, 120, 400, 60, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextSize(5);
  M5.Display.setCursor(20, 120);
  M5.Display.printf("%lu", (unsigned long)n);

  n++;
  delay(1000);
}
