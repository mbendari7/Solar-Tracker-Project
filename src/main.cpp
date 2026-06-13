/*
  DUAL-AXIS SOLAR TRACKER + POWER LOGGER  -  Arduino Nano ESP32

  What this does, in plain terms:
  Four light sensors sit behind a small plus-shaped wall. The wall casts a
  shadow, so whichever side faces away from the sun reads darker. Comparing the
  four readings tells the tracker which way the light is, and two servos turn the
  panel to face it. While that happens, an INA219 sensor watches the solar
  panel's output and a colour screen shows the live power and the running total
  of energy collected.

  How the pins are used:
  light sensors  TL / TR / BL / BR  ->  A0 / A1 / A2 / A3   (3.3 V only)
  pan servo  -> D2        tilt servo -> D3                  (5 V from the MB102)
  INA219  (I2C)  SDA -> A4   SCL -> A5   VCC -> 3V3   GND -> GND
  panel current passes through VIN+ and VIN-
  ST7735 screen (SPI)  SCK -> D13  MOSI -> D11  CS -> D10
  DC -> D9    RES -> D8    BL -> 3V3
  one shared ground ties the whole thing together
*/

#include <Wire.h>            // I2C bus, used by the INA219 power sensor
#include <SPI.h>             // SPI bus, used by the screen
#include <Adafruit_GFX.h>    // shared drawing commands (text, shapes)
#include <Adafruit_ST7735.h> // driver for this exact 1.8" colour screen
#include <Adafruit_INA219.h> // reads voltage, current and power from the panel
#include <ESP32Servo.h>      // servo control that works on the ESP32 chip

// --- light sensors: top-left, top-right, bottom-left, bottom-right ---
const int PIN_TL = A0, PIN_TR = A1, PIN_BL = A2, PIN_BR = A3;

// --- servo signal pins ---
const int PAN_PIN = D2, TILT_PIN = D3;

// --- screen control pins (clock and data are the hardware SPI pins D13/D11) ---
#define TFT_CS D10 // chip select: tells the screen to listen
#define TFT_DC D9  // data / command: marks each byte as a value or an order
#define TFT_RST D8 // reset line
#define TFT_BL D7  // backlight on/off (a direct wire to 3V3 also works)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

Adafruit_INA219 ina219;    // the power sensor object
Servo panServo, tiltServo; // the two servo objects

// --- behaviour tuning, all in one place for easy changes ---
const int DEADZONE = 60;                   // how unequal the sides must be before moving (ignores noise)
const int STEP_DEG = 1;                    // degrees moved per loop, kept small for smooth motion
const int SETTLE_MS = 20;                  // short pause each loop so the servos can keep up
const int ANGLE_MIN = 10, ANGLE_MAX = 170; // safe travel limits for both servos
const int PAN_DIR = +1, TILT_DIR = +1;     // flip a sign here if a servo turns the wrong way

// --- live values the program keeps track of ---
int panAngle = 90, tiltAngle = 90; // current servo positions, start centred
int tl, tr, bl, br, hErr, vErr;    // sensor readings and the two error sums

// --- power and energy ---
float busV = 0, current_mA = 0, power_mW = 0; // latest readings from the INA219
double energy_mWh = 0;                        // running total of energy collected
unsigned long lastEnergyMs = 0;               // timestamp used to measure each slice

// --- screen refresh timing ---
unsigned long lastDrawMs = 0;
const unsigned long DRAW_EVERY = 400; // redraw about 2-3 times a second to avoid flicker

void readSensors();
void updateDisplay();

void setup()
{
  Serial.begin(115200);     // opens the USB text channel for debugging
  analogReadResolution(12); // the ESP32 reads 0-4095 instead of 0-1023

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // turn the backlight on

  // start the power sensor on the I2C bus
  Wire.begin();
  if (!ina219.begin())
    Serial.println("INA219 not found - check wiring");

  // wake up the screen, set it to landscape, and show a short splash
  tft.initR(INITR_BLACKTAB); // colours look wrong? swap for GREENTAB or REDTAB
  tft.setRotation(3);        // landscape, 160 wide by 128 tall
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(8, 44);
  tft.print("SOLAR");
  tft.setCursor(8, 66);
  tft.print("TRACKER");
  delay(900);

  // reserve the ESP32 timers the servo library needs, then attach the servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50); // standard servo rate
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle); // move to the centre

  lastEnergyMs = millis(); // mark the starting time for energy counting
  delay(300);
}

void loop()
{
  // 1) read the light and work out which way to turn
  readSensors();
  hErr = ((tl + bl) / 2) - ((tr + br) / 2); // left brightness minus right brightness
  vErr = ((tl + tr) / 2) - ((bl + br) / 2); // top brightness minus bottom brightness

  // nudge each servo one small step toward the brighter side, but only if the
  // difference is past the deadzone, which stops it twitching in steady light
  if (abs(hErr) > DEADZONE)
    panAngle += PAN_DIR * ((hErr > 0) ? STEP_DEG : -STEP_DEG);
  if (abs(vErr) > DEADZONE)
    tiltAngle += TILT_DIR * ((vErr > 0) ? STEP_DEG : -STEP_DEG);

  // keep both servos inside their safe range and send the new positions
  panAngle = constrain(panAngle, ANGLE_MIN, ANGLE_MAX);
  tiltAngle = constrain(tiltAngle, ANGLE_MIN, ANGLE_MAX);
  panServo.write(panAngle);
  tiltServo.write(tiltAngle);

  // 2) read the panel, then add this moment's contribution to the energy total
  busV = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  if (power_mW < 0)
    power_mW = 0; // clip tiny negative noise to zero
  unsigned long now = millis();
  // energy = power x time; this adds power times the fraction of an hour just elapsed
  energy_mWh += power_mW * ((now - lastEnergyMs) / 3600000.0);
  lastEnergyMs = now;

  // 3) refresh the screen on a timer so it stays readable instead of flickering
  if (now - lastDrawMs >= DRAW_EVERY)
  {
    updateDisplay();
    lastDrawMs = now;
  }

  delay(SETTLE_MS);
}

// grabs all four light readings in one place
void readSensors()
{
  tl = analogRead(PIN_TL);
  tr = analogRead(PIN_TR);
  bl = analogRead(PIN_BL);
  br = analogRead(PIN_BR);
}

// paints the whole screen: title, live power, total energy, voltage, and status
void updateDisplay()
{
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(4, 4);
  tft.print("SOLAR TRK");

  tft.setTextSize(1);
  int y = 34;
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(4, y);
  tft.print("Power now:");
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(86, y);
  tft.print(power_mW, 1);
  tft.print(" mW");
  y += 16;
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(4, y);
  tft.print("Energy:");
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(86, y);
  tft.print(energy_mWh, 3);
  tft.print(" mWh");
  y += 16;
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(4, y);
  tft.print("Voltage:");
  tft.setCursor(86, y);
  tft.print(busV, 2);
  tft.print(" V");
  y += 22;
  tft.setTextColor(0xFD20);
  tft.setCursor(4, y); // orange for the mechanics
  tft.print("Pan ");
  tft.print(panAngle);
  tft.print("  Tilt ");
  tft.print(tiltAngle);
  y += 16;
  tft.setTextColor(0x7BEF);
  tft.setCursor(4, y); // grey for the raw numbers
  tft.print("L ");
  tft.print(tl);
  tft.print(" ");
  tft.print(tr);
  tft.print(" ");
  tft.print(bl);
  tft.print(" ");
  tft.print(br);
}