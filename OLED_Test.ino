#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void drawHeart(int x, int y) {
  display.fillCircle(x, y, 3, SSD1306_WHITE);
  display.fillCircle(x + 6, y, 3, SSD1306_WHITE);
  display.fillTriangle(x - 3, y + 2, x + 9, y + 2, x + 3, y + 10, SSD1306_WHITE);
}

void setup() {
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(10, 12);
  display.println("TEAM");

  display.setCursor(0, 36);
  display.print("TAFALING");

  drawHeart(104, 43);   // Heart at the end

  display.display();
}

void loop() {
}
