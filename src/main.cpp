/*
  Dual-Axis Solar Tracker - Arduino Nano ESP32

  Four light sensors (one per corner) sit behind a + shaped wall.
  The firmware compares left vs right and top vs bottom brightness,
  then drives two servos until all four read roughly equal, so the
  head ends up facing the brightest light. As the sun moves, the
  balance breaks and the tracker re-centers.

  Wiring:
    sensors TL/TR/BL/BR -> A0/A1/A2/A3   (VCC to 3V3 only, not 5V)
    OLED  SDA->A4  SCL->A5  VCC->3V3  GND->GND
    pan servo -> D2   tilt servo -> D3   (servo +5V from the MB102)
    common ground shared across all parts
*/

#include <Wire.h>             // I2C bus for the OLED
#include <Adafruit_GFX.h>     // graphics primitives
#include <Adafruit_SSD1306.h> // OLED driver
#include <ESP32Servo.h>       // ESP32 servo library (not Servo.h)

// Pin assignments - adjust to match the wiring
const int PIN_TL = A0; // top-left sensor
const int PIN_TR = A1; // top-right
const int PIN_BL = A2; // bottom-left
const int PIN_BR = A3; // bottom-right

const int PAN_PIN = D2;  // pan servo (left/right)
const int TILT_PIN = D3; // tilt servo (up/down)

// OLED configuration
#define SCREEN_W 128
#define SCREEN_H 64    // use 32 for the 128x32 panel
#define OLED_ADDR 0x3C // try 0x3D if the screen stays blank
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

Servo panServo;
Servo tiltServo;

// Tuning constants
const int DEADZONE = 60;  // ignore imbalances smaller than this
const int STEP_DEG = 1;   // degrees moved per correction
const int SETTLE_MS = 20; // pause after each move
const int ANGLE_MIN = 10; // stay clear of the servo end-stops
const int ANGLE_MAX = 170;
const int PAN_DIR = +1;  // set to -1 to reverse pan direction
const int TILT_DIR = +1; // set to -1 to reverse tilt direction

// Current servo positions
int panAngle = 90;
int tiltAngle = 90;

// Sensor readings and the two balance errors
int tl, tr, bl, br;
int hErr, vErr;

// Forward declarations (defined below)
void readSensors();
void updateDisplay();

// Runs once at startup
void setup()
{
  Serial.begin(115200);
  analogReadResolution(12); // full 0..4095 ADC range

  // Initialize the OLED
  Wire.begin(); // I2C on A4/A5
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("OLED not found - check wiring/address");
    while (true)
    {
    } // halt so the fault is obvious
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Solar Tracker");
  display.println("starting...");
  display.display();
  delay(1000);

  // Initialize the servos
  ESP32PWM::allocateTimer(0); // reserve the ESP32 PWM timers
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);

  panServo.write(panAngle); // move to center
  tiltServo.write(tiltAngle);
  delay(500);
}

// Main control loop
void loop()
{
  readSensors();

  // Convert readings into horizontal/vertical imbalance
  // (+hErr = left brighter, +vErr = top brighter)
  hErr = ((tl + bl) / 2) - ((tr + br) / 2);
  vErr = ((tl + tr) / 2) - ((bl + br) / 2);

  // Move only if the imbalance exceeds the deadzone
  if (abs(hErr) > DEADZONE)
  {
    panAngle += PAN_DIR * ((hErr > 0) ? STEP_DEG : -STEP_DEG);
  }
  if (abs(vErr) > DEADZONE)
  {
    tiltAngle += TILT_DIR * ((vErr > 0) ? STEP_DEG : -STEP_DEG);
  }

  // Clamp to a safe range, then drive the servos
  panAngle = constrain(panAngle, ANGLE_MIN, ANGLE_MAX);
  tiltAngle = constrain(tiltAngle, ANGLE_MIN, ANGLE_MAX);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  updateDisplay();
  delay(SETTLE_MS);
}

// Read all four sensors (0 = dark, 4095 = bright)
void readSensors()
{
  tl = analogRead(PIN_TL);
  tr = analogRead(PIN_TR);
  bl = analogRead(PIN_BL);
  br = analogRead(PIN_BR);
}

// Show live readings, errors, and angles on the OLED
void updateDisplay()
{
  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("TL ");
  display.print(tl);
  display.print("  TR ");
  display.println(tr);
  display.print("BL ");
  display.print(bl);
  display.print("  BR ");
  display.println(br);
  display.print("Herr ");
  display.println(hErr);
  display.print("Verr ");
  display.println(vErr);
  display.print("Pan ");
  display.print(panAngle);
  display.print(" Tilt ");
  display.println(tiltAngle);

  display.display();
}