#include <TouchScreen.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include "max6675.h"
#include <EEPROM.h>
#include "PID961.h"
#include <avr/wdt.h>  // Include the avr-libc Watchdog timer handling library



//#define DEBUGEEE

// Pinouts
#define relay 11
#define PIDSAMPLETIME 1000

#define FIRST_BUTTON_Y 75

#define SECONDSTOSAVE 30

#define thermoCS 10
#define thermoDO 12
#define thermoCLK 13

#define PROPERTIES_COUNT 6  //TEMP SETPOINT Kp Ki Kd


#define CHECKSUM_ADRESS 0
#define CHECKSUM 123
#define CONFIG_ADRESS 10



// Variables
//uint8_t redraw_counter = 0; //counter used to redraw the screen
unsigned long previousMillis = 0;  //counter for every
double Actualtemp = 0;             //current temperature read
double Output = 0;
struct config_t {
  double Setpoint;
  double Kp;
  double Ki;
  double Kd;
};
config_t current_config = { .Setpoint = 100, .Kp = 10, .Ki = 1, .Kd = 10 };

uint8_t last_change = SECONDSTOSAVE;

int x = 165,
    y = 231, w = 36, h = 36;

MCUFRIEND_kbv tft;

PID myPID(&Actualtemp, &Output, &current_config.Setpoint, 0, 0, 0, DIRECT);
unsigned long windowStartTime;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// require touch screen calibration
// Put your calibrated touchSensor values here
const int XP = 8, XM = A2, YP = A3, YM = 9;  //240x320 ID=0x9341

const int TS_LEFT = 139, TS_RT = 895, TS_TOP = 907, TS_BOT = 91;

#define MINPRESSURE 200
#define MAXPRESSURE 1000

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


// get touch input
int pixel_x, pixel_y;
bool Touch_getXY(void) {
  TSPoint p = ts.getPoint();
  pinMode(YP, OUTPUT);  //restore shared pins
  pinMode(XM, OUTPUT);
  digitalWrite(YP, HIGH);  //because TFT control pins
  digitalWrite(XM, HIGH);
  bool pressed = (p.z > MINPRESSURE && p.z < MAXPRESSURE);
  if (pressed) {
#ifdef DEBUGEEE
    Serial.println(String(p.x) + " " + String(p.y));
#endif
    pixel_x = map(p.x, TS_LEFT, TS_RT, 0, tft.width());
    pixel_y = map(p.y, TS_TOP, TS_BOT, 0, tft.height());
  }
  return pressed;
}

#define BLACK 0x0000
#define WHITE 0xFFFF
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define GRAY 0x6B4D


struct property_t {
  double* value_ptr;
  uint16_t min;
  uint16_t max;
  uint8_t inc;
  const char* name;
  Adafruit_GFX_Button* btnp;
  Adafruit_GFX_Button* btnm;
  void (*callback)(void);
};
property_t properties[PROPERTIES_COUNT] = {
  { .value_ptr = &Actualtemp, .min = -10, .max = 500, .inc = 0, .name = "Temp" },
  { .value_ptr = &current_config.Setpoint, .min = 0, .max = 400, .inc = 10, .name = "Set" },
  { .value_ptr = &current_config.Kp, .min = 0, .max = 100, .inc = 1, .name = "Kp" },
  { .value_ptr = &current_config.Ki, .min = 0, .max = 100, .inc = 1, .name = "Ki" },
  { .value_ptr = &current_config.Kd, .min = 0, .max = 100, .inc = 1, .name = "Kd" },
  { .value_ptr = &Output, .min = 0, .max = 100, .inc = 0, .name = "Pwr" }
};

void setup(void) {
  wdt_enable(WDTO_4S);  // Set watchdog timeout to 4 seconds
  wdt_reset();          // Reset the watchdog timer
  windowStartTime = millis();
#ifdef DEBUGEEE
  Serial.begin(115200);
#endif
  pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);
  tft.begin(0x9341);
  tft.setRotation(2);
  tft.fillScreen(BLACK);
  tft.fillRect(45, 0, 156, 5, WHITE);
  tft.fillRect(45, 20, 156, 5, WHITE);
  tft.setTextColor(BLACK, WHITE);
  tft.setTextSize(2);
  tft.setCursor(45, 5);
  tft.print(" REFLOW IRON ");

  loadConfig();  //LOAD CONFIG

  tft.setTextColor(BLACK, YELLOW);
  // initialise buttons
  for (uint8_t i = 0; i < PROPERTIES_COUNT; i++) {
    redrawButtonText(i, true);
    if (properties[i].inc != 0) {
      properties[i].btnp = (Adafruit_GFX_Button*)malloc(sizeof(Adafruit_GFX_Button));
      properties[i].btnm = (Adafruit_GFX_Button*)malloc(sizeof(Adafruit_GFX_Button));
      properties[i].btnp->initButton(&tft, 190, FIRST_BUTTON_Y + (35 * i), 25, 25, WHITE, BLACK, WHITE, "+", 2);
      properties[i].btnm->initButton(&tft, 220, FIRST_BUTTON_Y + (35 * i), 25, 25, WHITE, BLACK, WHITE, "-", 2);
      properties[i].btnp->drawButton(false);
      properties[i].btnm->drawButton(false);
      if (i > 1) properties[i].callback = &handlePidChange;
    }
  }

  myPID.SetOutputLimits(0, 100);                                              //tell the PID to range between 0 and the full window size
  myPID.SetTunings(current_config.Kp, current_config.Ki, current_config.Kd);  //SET AGRESSIVE TUNING PARAMETERS
  myPID.SetMode(AUTOMATIC);                                                   // ACTIVATE PIDF CONTROLLER
  myPID.SetSampleTime(PIDSAMPLETIME);
}
void handlePidChange() {
  myPID.SetTunings(current_config.Kp, current_config.Ki, current_config.Kd);  //SET AGRESSIVE TUNING PARAMETERS
}
void redrawButtonText(uint8_t i, bool fresh) {
  tft.fillRect(0, (FIRST_BUTTON_Y + 15) + (35 * i), 240, 3, GRAY);

  tft.fillRect(90, (FIRST_BUTTON_Y - 5) + (35 * i), 80, 20, BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  if (fresh) {
    tft.setCursor(10, (FIRST_BUTTON_Y - 5) + (35 * i));
    tft.print(properties[i].name);
    tft.print(":");
  }
  tft.setCursor(90, (FIRST_BUTTON_Y - 5) + (35 * i));
  tft.print(*properties[i].value_ptr);
}
void loop(void) {
  wdt_reset();  // Reset the watchdog timer
  myPID.Compute();

  bool down = Touch_getXY();

  for (uint8_t i = 0; i < PROPERTIES_COUNT; i++) {
    // PreHeat
    if (properties[i].inc == 0) continue;
    properties[i].btnp->press(down && properties[i].btnp->contains(pixel_x, pixel_y));
    properties[i].btnm->press(down && properties[i].btnm->contains(pixel_x, pixel_y));


    if (properties[i].btnp->justPressed()) properties[i].btnp->drawButton(true);
    if (properties[i].btnm->justPressed()) properties[i].btnm->drawButton(true);
    if (properties[i].btnp->justReleased()) {
      properties[i].btnp->drawButton();
      if (*properties[i].value_ptr < properties[i].max) {
        *properties[i].value_ptr += (properties[i].inc / 10.0);
        if (properties[i].callback != NULL)
          properties[i].callback();
        redrawButtonText(i, false);
        last_change = 0;
      }
    }
    if (properties[i].btnm->justReleased()) {
      properties[i].btnm->drawButton();
      if (*properties[i].value_ptr > properties[i].min) {
        *properties[i].value_ptr -= (properties[i].inc / 10.0);
        if (properties[i].callback != NULL)
          properties[i].callback();
        redrawButtonText(i, false);
        last_change = 0;
      }
    }
  }
  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();
    if (last_change < SECONDSTOSAVE) {
      last_change++;
      if (last_change == SECONDSTOSAVE)
        saveConfig();
    }

    Actualtemp = thermocouple.readCelsius();  // get temperature from thermocouple

    if (Actualtemp != Actualtemp)
      showError(1);
    else if (Actualtemp < -30)
      showError(2);
    else if (Actualtemp > 500)
      showError(3);


    redrawButtonText(0, false);  //redraw temperature
    redrawButtonText(5, false);  //redraw temperature

#ifdef DEBUGEEE
    Serial.println();
    Serial.println("Setpoint:");
    Serial.println(current_config.Setpoint);
    Serial.println("CurrentT:");
    Serial.println(Actualtemp);
    Serial.println("Power:");
    Serial.println(Output);
#endif
  }

  if (millis() - windowStartTime > PIDSAMPLETIME) {  //time to shift the Relay Window
    windowStartTime += PIDSAMPLETIME;
  }

  digitalWrite(relay, (map(Output, 0, 100, 0, PIDSAMPLETIME) > (millis() - windowStartTime)));
}

void saveConfig() {
#ifdef DEBUGEEE
  Serial.println("Saving EEPROM");
#endif
  EEPROM.put(CONFIG_ADRESS, current_config);
  uint8_t checksum = 0;
  EEPROM.get(CHECKSUM_ADRESS, checksum);
  if (checksum != CHECKSUM) {
    checksum = CHECKSUM;
    EEPROM.put(CHECKSUM_ADRESS, checksum);
  }
}
void loadConfig() {
#ifdef DEBUGEEE
  Serial.println("Loading EEPROM");
#endif
  uint8_t checksum = 0;
  EEPROM.get(CHECKSUM_ADRESS, checksum);
  if (checksum == CHECKSUM)
    EEPROM.get(CONFIG_ADRESS, current_config);
  else {
#ifdef DEBUGEEE
    Serial.println("Initialising EEPROM");
#endif
    saveConfig();
  }
}
void showError(uint8_t err) {
  digitalWrite(relay, LOW);
  tft.fillScreen(BLACK);
  tft.setTextColor(YELLOW, RED);
  tft.setTextSize(5);
  tft.setCursor(10, 100);
  tft.print("ERROR ");
  tft.print(err);
  while (true) {
    wdt_reset();  // Reset the watchdog timer
    delay(1000);
  }
}