/* Code for TV panel (without the LCD) being used as a light.
So far we have 
- Solar elevation.
- Two different methods of mapping brightness based on elevation of sun.
- A LED brightness curve that really only works in 8-bit mode. Math doesn't seem to work as well with 16-bit PWM. 
- 16-bit PWM - MUCH smoother!
- Switch that can disable Skylight mode and read a potentiometer instead.

Designed for Board: Arduino Uno
*/

// #include <LiquidCrystal.h>
#include <EEPROM.h>
#include <SolarCalculator.h>
#include <RTClib.h>
#include <Wire.h>
#include <Math.h>
#include <Adafruit_SSD1306.h>
RTC_DS3231 rtc;

// Configuration Variables
const float latitude = 34.21316116274671;   // Hardcode your current location for skylight mode
const float longitude = -84.54894616203148; // Hardcode your current location for skylight mode
#define DAYLIGHT_VALUE 200                  // Compared to full range of 0-255 - my light is way too bright for indoors 
#define DAYLIGHT_VALUE16 55000
#define MIN_ILLUM 1
#define MIN_ILLUM16 25                      // Because some displays really don't like being super low on the PWM signal.
#define PHOTO_OFFSET -50                    // The photoresistors aren't identical, and their connections and cord lengths aren't identical. The external reading will be adjusted by this much.
#define DIMMING_TIME 30000                  // OLED dimming time in milliseconds (30 seconds)
#define DIMMED_BRIGHTNESS 5                 // OLED dimmed brightness as a percentage (5%)
#define OFF_TIME 300000                     // OLED Off time in milliseconds (5 minutes)
#define SKY_CHECK_INTERVAL 10000            // Used in mode 1&2 - only update skylight every xx milliseconds
#define OLED_UPDATE_INTERVAL 500            // Milliseconds between OLED display updates

// Enabled Modes
#define MODE_POTENTIOMETER                  // Only include the modes you want available
#define MODE_SKYLIGHT1
#define MODE_SKYLIGHT2
//#define MODE_PHOTO_MATCH
//#define MODE_DEMO

// Pin Designations
#define LED_PIN 11
#define LED_PIN16 9
#define MODE_SW 8
#define PHOTO_INT_PIN A2
#define PHOTO_EXT_PIN A1
#define POT_PIN A0

// OLED Setup
#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 32        // OLED display height, in pixels
#define OLED_RESET    -1        // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Debugging setup
#define DEBUG_NONE 0
#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_INFO 3
#define DEBUG_VERBOSE 4
#define DEBUG_NEVER 9 // This is really only a placeholder. If you want to see this information, change the debug level on the specific line of code from NEVER to something else.

#define DEBUG_LEVEL DEBUG_WARNING       // Set the debug level here

#define DEBUG_PRINT(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.print(message); } while (0)
#define DEBUG_PRINTLN(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.println(message); } while (0)

// EEPROM addresses
#define ADDR_SIGNATURE 0                // Address for signature
#define ADDR_MODE 1                     // Address for the mode
#define EEPROM_SIGNATURE 0xD4           // Signature byte

// Global Scope Declarations
unsigned long currentMillis = 0;        // stores the value of millis() in each iteration of loop()
unsigned long previousMillis = 0;       // for various timings - mode dependent
unsigned long lastInteractionMillis = 0;// Global variable to track the last interaction time for OLED dimming
unsigned long lastOledUpdateMillis = 0; // Global variable to limit OLED updates
uint8_t curmode = 0;                    // 0-Manual, 1-Skylight1, 2-Skylight2, 3-PhotoMatch, 4-Demo
uint8_t led_value = 10;                 // This will be the dynamic LED brightness sent to PWM
uint16_t led_value16 = MIN_ILLUM16;     // 16-bit version. Only keeping the 8-bit for legacy compatibility
#define ICR 0xffff                      // I don't really understand this but it's used in setting up 16-bit PWM
bool isDimmed = false;                  // Variable to track the current display brightness state
bool isOff = false;                     // Variable to track the current display brightness state

// Global variables for MODE_POTENTIOMETER
#if defined(MODE_POTENTIOMETER)
int potval;                             // Used in mode 0 - value read from the potentiometer on pot_pin
int previousPotval = 0;                 // Used to change to manual mode from other modes
#define AVERAGE_BUFFER_SIZE 100         // Number of readings to average
int potReadings[AVERAGE_BUFFER_SIZE];   // Array to store the potentiometer readings
int readIndex = 0;                      // Index for the next reading
long total = 0;                         // Running total of the readings
int average = 0;                        // Average of the readings
#endif

// Global variables used in Skylight Mode 1
#if defined(MODE_SKYLIGHT1)
float relative_brightness;
float relative_brightness16;
#endif

// Global variables used in Skylight Mode 2
#if defined(MODE_SKYLIGHT2)
#define MATCH_ELEVATION 1                // Elevation angle at which we switch between twilight and daylight models. Don't recommend you change it.
#define ASTRONOMICAL_DECAY_CONSTANT 0.15 // For how slow the twilight transformation goes
#define DIFFUSE_SCALING_FACTOR 1         // Will be multiplied. 1 means no change. Higher emphasises the impact of ambient reflection of light
float match_factor;
float max_combined_level = 0;
#endif

// Global variables used in Photo Match Mode
#if defined(MODE_PHOTO_MATCH)
int photo_int;                          // Used in mode 3 - value read from the indoor photoresistor
int photo_ext;                          // Used in mode 3 - value read from the outdoor photoresistor
#endif

// Global variables used in Demo Mode
#if defined(MODE_DEMO)
boolean demoDirectionUp = true;         // Used in mode 4 - state variable
uint8_t demoCounter = 255;              // Used in mode 4 - increment the 8-bit PWM value once every 256 iterations of the 16-bit loop
#endif

// Function Prototypes
void handlePotentiometerMode(bool forceUpdate = false);
void handleSkylightMode1(bool forceUpdate = false);
void handleSkylightMode2(bool forceUpdate = false);
void handlePhotoresistorMatchMode(bool forceUpdate = false);
void handleDemoMode(bool forceUpdate = false);
unsigned long getNextDemoInterval(uint16_t led_value16, bool goingUp);
void initializeEEPROM();
void setupPWM16();
void analogWrite16(uint8_t pin, uint16_t val);
void readButton();
void cycleModes();
void enterTimeSettingMode();
void adjustTimeWithPot();
void displayTimeSetting(const DateTime &now);
void updateDisplay(const char* mode, float elevation, uint16_t led_value16, bool forceUpdate = false);
uint8_t gammaCorrection(uint8_t led_value, float gamma = 2.2);
uint16_t gammaCorrection16(uint16_t led_value16, float gamma = 2.2);

// *** SETUP ***
void setup() {
  Serial.begin(115200);
  Serial.println("Serial started");
  DEBUG_PRINTLN(DEBUG_INFO, "Setup started");

  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_SW, INPUT_PULLUP);
  setupPWM16();
  
  // Restore last mode
  initializeEEPROM();
  curmode = EEPROM.read(ADDR_MODE);
  DEBUG_PRINT(DEBUG_INFO, "Setting initial Mode from EEPROM: ");
  DEBUG_PRINTLN(DEBUG_INFO, curmode);
  
  // Time Initialization
  if (!rtc.begin()) {
    DEBUG_PRINTLN(DEBUG_ERROR, "Couldn't find RTC");
    Serial.flush();
  }

  // OLED Begin
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    DEBUG_PRINTLN(DEBUG_ERROR, "SSD1306 allocation failed");
    Serial.flush();
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.cp437(true);
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(255); // Set initial brightness to 100%
  
  // Used in MODE_POTENTIOMETER
  #if defined(MODE_POTENTIOMETER)
  for (int thisReading = 0; thisReading < AVERAGE_BUFFER_SIZE; thisReading++) { // Init Debounce Array
    potReadings[thisReading] = 0; 
  }
  previousPotval = analogRead(POT_PIN); // So we don't immediately switch to manual on first run.
  #endif

  // Used in Skylight Mode 1 - Linear LED Brightness
  #if defined(MODE_SKYLIGHT1)
  relative_brightness = (255 * log10(2)) / log10(255); // https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
  relative_brightness16 = (65535 * log10(2)) / log10(65535);
  #endif

  // Used in Skylight Mode 2 - Calculate combined and twilight light intensities at a matching point
  #if defined(MODE_SKYLIGHT2)
  // Calculate the match factor - so we have a smooth transition between twilight and daylight modes
  match_factor = combined_daytime_brightness(MATCH_ELEVATION + 0.001) / astronomical_twilight_brightness(MATCH_ELEVATION - 0.001);
  
  // Calculate the max and min values of the output of our curve
  for (float elevation = -20; elevation <= 90; elevation += 1) {
      float combined_level = combined_brightness(elevation);
      if (combined_level > max_combined_level) {
          max_combined_level = combined_level;
      }
  }
  #endif

  // Track initial interaction time
  lastInteractionMillis = millis();

  Serial.println("Ending Setup()");
  Serial.flush();
}

// *** MAIN LOOP ***
void loop() {  
  currentMillis = millis();   // capture the latest value of millis()
  
  // Start by looking for a mode change
  readButton(); // looks for mode change switch use
  
  // Potential mode change through potentiometer 
  #if defined(MODE_POTENTIOMETER)
  potval = analogRead(POT_PIN); // doing this now so I can use the value to check for mode change, and later if I'm in manual mode
  int potChange = abs(potval - previousPotval);
  if (curmode != 0 && potChange > 50) {
    curmode = 0; // Dimmer moved - switch to manual dimming mode
    previousMillis = 0; // Always reset on mode change
    DEBUG_PRINTLN(DEBUG_WARNING, "Manual Overide - Entering mode 0 - Manual Dimming");
    EEPROM.update(ADDR_MODE, curmode);  // Save new mode to EEPROM whenever it changes
    previousPotval = potval;
  }
  #else 
  int potChange = 0;
  #endif 

  // Reset the last interaction time if there is any interaction
  if (potChange > 25 || digitalRead(MODE_SW) == LOW) {
    lastInteractionMillis = currentMillis;
    if (isDimmed) {
      display.ssd1306_command(SSD1306_SETCONTRAST);
      display.ssd1306_command(255);
      isDimmed = false;
    }
    if (isOff) {
      display.display();
      isOff = false;
    }
  }
  
  // Dimming logic
  if ((currentMillis - lastInteractionMillis >= DIMMING_TIME) && !isDimmed) {
    DEBUG_PRINTLN(DEBUG_INFO, "Dimming OLED now.");
    uint8_t correctedDimmedBrightness = gammaCorrection(map(DIMMED_BRIGHTNESS, 0, 100, 0, 255));
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(correctedDimmedBrightness);
    isDimmed = true;
  }

  // Off timer logic
  if ((currentMillis - lastInteractionMillis >= OFF_TIME) && !isOff) {
    DEBUG_PRINTLN(DEBUG_INFO, "Turning off OLED now.");
    display.clearDisplay();
    display.display();
    isOff = true;
  }
  
  switch (curmode) {
    case 0:
      handlePotentiometerMode();
      break;
    case 1:
      handleSkylightMode1();
      break;
    case 2:
      handleSkylightMode2();
      break;
    case 3:
      handlePhotoresistorMatchMode();
      break;
    case 4:
      handleDemoMode();
      break;
  }
}

// Basic manual dimmer mode with gamma correction
void handlePotentiometerMode(bool forceUpdate) {
  #if defined(MODE_POTENTIOMETER)
  // Debounce
  total = total - potReadings[readIndex];            // Subtract the oldest reading from the total
  potReadings[readIndex] = potval;                   // Add the current potentiometer value and store it into the array
  total = total + potReadings[readIndex];            // Add the new reading to the total
  readIndex = (readIndex + 1) % AVERAGE_BUFFER_SIZE; // Advance to the next position in the array
  average = total / AVERAGE_BUFFER_SIZE;             // Calculate the average
  
  // Map the average potentiometer value to the 8-bit and 16-bit ranges
  uint8_t mapped_value = map(average, 0, 1023, 0, 255);
  uint16_t mapped_value16 = map(average, 0, 1023, 0, 65535);
  
  // Apply gamma correction
  led_value = gammaCorrection(mapped_value);
  led_value16 = gammaCorrection16(mapped_value16);

  // Write the corrected values to the LED pins
  analogWrite(LED_PIN, led_value);
  analogWrite16(LED_PIN16, led_value16);
  updateDisplay("Manual Dimming", -1, led_value16, forceUpdate);

  // Update debug, based on timing
  if (currentMillis - previousMillis > 500 || forceUpdate == true) { // Only debug every half second
    previousMillis = currentMillis;
    
    // Consolidated debug output
    DEBUG_PRINT(DEBUG_VERBOSE, "Potval: ");
    DEBUG_PRINT(DEBUG_VERBOSE, potval);
    DEBUG_PRINT(DEBUG_VERBOSE, " | Avg: ");
    DEBUG_PRINT(DEBUG_VERBOSE, average);
    DEBUG_PRINT(DEBUG_VERBOSE, " | Mapped 8-bit: ");
    DEBUG_PRINT(DEBUG_VERBOSE, mapped_value);
    DEBUG_PRINT(DEBUG_VERBOSE, " | Mapped 16-bit: ");
    DEBUG_PRINT(DEBUG_VERBOSE, mapped_value16);
    DEBUG_PRINT(DEBUG_VERBOSE, " | LED 8-bit: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value);
    DEBUG_PRINT(DEBUG_VERBOSE, " | LED 16-bit: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);  
  }
  #endif
}

// *** Artificial Skylight Mode 1 - Modeled
void handleSkylightMode1(bool forceUpdate) {
  #if defined(MODE_SKYLIGHT1)
  if (currentMillis - previousMillis >= SKY_CHECK_INTERVAL || forceUpdate == true) {
    previousMillis = currentMillis;
    DEBUG_PRINTLN(DEBUG_INFO, "In Skylight Mode 1");

    // Get the sun's current position
    double azimuth, elevation;
    DateTime now = rtc.now();
    calcHorizontalCoordinates(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), latitude, longitude, azimuth, elevation);

    DEBUG_PRINT(DEBUG_INFO, "Elevation value: ");
    DEBUG_PRINTLN(DEBUG_INFO, elevation);

    // Non-linear brightness growth of the sun based on elevation angle
    if (elevation >= 40) {
      led_value = DAYLIGHT_VALUE;
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = DAYLIGHT_VALUE16;
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 15) {
      led_value = map(elevation * 100, 15 * 100, 40 * 100, .85 * DAYLIGHT_VALUE, DAYLIGHT_VALUE);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, 15 * 100, 40 * 100, .85 * DAYLIGHT_VALUE16, DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 3) {
      led_value = map(elevation * 100, 3 * 100, 15 * 100, .76 * DAYLIGHT_VALUE, .85 * DAYLIGHT_VALUE);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, 3 * 100, 15 * 100, .76 * DAYLIGHT_VALUE16, .85 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 0) {
      led_value = map(elevation * 100, 0 * 100, 3 * 100, .7 * DAYLIGHT_VALUE, .76 * DAYLIGHT_VALUE);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, 0 * 100, 3 * 100, .7 * DAYLIGHT_VALUE16, .76 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= -3) {
      led_value = map(elevation * 100, -3 * 100, 0 * 100, .6 * DAYLIGHT_VALUE, .7 * DAYLIGHT_VALUE);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, -3 * 100, 0 * 100, .6 * DAYLIGHT_VALUE16, .7 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
      led_value16 += map(elevation * 100, -3 * 100, 0 * 100, MIN_ILLUM16, 0);
    } else if (elevation >= -12) {
      led_value = map(elevation * 100, -12 * 100, -3 * 100, 0, .6 * DAYLIGHT_VALUE);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, -12 * 100, -3 * 100, 0, .6 * DAYLIGHT_VALUE16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
      led_value16 += MIN_ILLUM16;
    } else {
      led_value = 0;
      led_value16 = 0;
    }

    // Guard rails
    if (led_value > 0 && led_value < MIN_ILLUM / 2) led_value = 0; // LED stays off until halfway to MIN_ILLUM
    else if (led_value >= MIN_ILLUM / 2 && led_value < MIN_ILLUM) led_value = MIN_ILLUM / 2; // LED turns on halfway
    if (led_value > DAYLIGHT_VALUE) led_value = DAYLIGHT_VALUE;
    if (led_value16 > 0 && led_value16 < MIN_ILLUM16 / 2) led_value16 = 0; // LED stays off until halfway to MIN_ILLUM16
    else if (led_value16 >= MIN_ILLUM16 / 2 && led_value16 < MIN_ILLUM16) led_value16 = MIN_ILLUM16 / 2; // LED turns on halfway
    if (led_value16 > DAYLIGHT_VALUE16) led_value16 = DAYLIGHT_VALUE16;

    // Write it
    analogWrite(LED_PIN, led_value);
    analogWrite16(LED_PIN16, led_value16);
    updateDisplay("Skylight Mode 1", elevation, led_value16, forceUpdate);

    DEBUG_PRINT(DEBUG_VERBOSE, "LED Value: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value);
    DEBUG_PRINT(DEBUG_VERBOSE, "   LED Value16: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);
  }
  #endif
}

// Artificial Skylight Mode 2
#if defined(MODE_SKYLIGHT2)
// Function to calculate the intensity of direct sunlight based on solar elevation using the Solar Air Mass model
float direct_sunlight_brightness(float elevation, float I0 = 1361, float k = 0.2) {
    if (elevation > MATCH_ELEVATION) {
        float zenith_angle = 90 - elevation;
        float AM = 1 / (cos(radians(zenith_angle)) + 0.50572 * pow((96.07995 - zenith_angle), -1.6364));
        return I0 * exp(-k * AM);
    } else {
        return 0;
    }
}

// Function to calculate diffuse sky light using the simplified Perez model
float diffuse_sky_brightness(float elevation, float a = -1, float b = -0.32, float c = 0.43, float d = 1.25, float e = -0.35) {
    if (elevation < MATCH_ELEVATION) {
        return 0;
    } else {
        float zenith_luminance = 100; // Assumed zenith luminance for a typical clear day
        return zenith_luminance * (1 + a * exp(b / (cos(radians(90 - elevation)) + 0.01)));
    }
}

// Function to calculate sky brightness during astronomical twilight using an exponential decay model
float astronomical_twilight_brightness(float elevation) {
    if (elevation > MATCH_ELEVATION) {
        return 0; // Daylight, no twilight effect
    } else {
        // Exponential decay of light intensity from sunset to end of astronomical twilight
        return exp(ASTRONOMICAL_DECAY_CONSTANT * elevation); // Adjust the decay rate to fit observational data
    }
}

// Function to scale and combine direct and diffuse daytime light components
float combined_daytime_brightness(float elevation) {
    float direct_component = direct_sunlight_brightness(elevation);
    float diffuse_component = diffuse_sky_brightness(elevation);
    return direct_component + (diffuse_component * DIFFUSE_SCALING_FACTOR);
}

// Function to scale and combine the twilight and daylight components
float combined_brightness(float elevation) {
    float combined_daytime_level = combined_daytime_brightness(elevation);
    float astronomical_twilight_level = astronomical_twilight_brightness(elevation);
    return combined_daytime_level + (astronomical_twilight_level * match_factor); // Match factor is determined in the setup() function.
}
#endif

// And the main handler for this mode
void handleSkylightMode2(bool forceUpdate) {
  #if defined(MODE_SKYLIGHT2)
  if (currentMillis - previousMillis >= SKY_CHECK_INTERVAL || forceUpdate == true) {
    previousMillis = currentMillis;
    double azimuth, elevation;
    DEBUG_PRINTLN(DEBUG_INFO, "In Skylight Mode 2");

    // Get the sun's current position
    DateTime now = rtc.now();
    calcHorizontalCoordinates(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), latitude, longitude, azimuth, elevation);
    DEBUG_PRINT(DEBUG_INFO, "Elevation value: ");
    DEBUG_PRINTLN(DEBUG_INFO, elevation);

    // Using a whole series of nifty functions above to create a pretty great curve, I think!
    float combined_level = combined_brightness(elevation);

    // Map the result to our dimmer range
    led_value = map(combined_level, 0, max_combined_level, 0, 255);
    led_value16 = map(combined_level, 0, max_combined_level, 0, 65535); // map algorithm value to 16-bit value

    // Gamma Correction
    led_value = gammaCorrection(led_value);
    led_value16 = gammaCorrection16(led_value16);

    // Guard rails
    if (led_value > 0 && led_value < MIN_ILLUM / 2) led_value = 0; // LED stays off until halfway to MIN_ILLUM
    else if (led_value >= MIN_ILLUM / 2 && led_value < MIN_ILLUM) led_value = MIN_ILLUM / 2; // LED turns on halfway
    if (led_value > DAYLIGHT_VALUE) led_value = DAYLIGHT_VALUE;
    if (led_value16 > 0 && led_value16 < MIN_ILLUM16 / 2) led_value16 = 0; // LED stays off until halfway to MIN_ILLUM16
    else if (led_value16 >= MIN_ILLUM16 / 2 && led_value16 < MIN_ILLUM16) led_value16 = MIN_ILLUM16 / 2; // LED turns on halfway
    if (led_value16 > DAYLIGHT_VALUE16) led_value16 = DAYLIGHT_VALUE16;
    
    // Write it
    analogWrite(LED_PIN, led_value);
    analogWrite16(LED_PIN16, led_value16);
    updateDisplay("Skylight Mode 2", elevation, led_value16, forceUpdate);

    DEBUG_PRINT(DEBUG_VERBOSE, "LED Value: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value);
    DEBUG_PRINT(DEBUG_VERBOSE, "   LED Value16: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);
  }
  #endif
}

// Photo Matching Mode
void handlePhotoresistorMatchMode(bool forceUpdate) {
  #if defined(MODE_PHOTO_MATCH)
  int photo_int = analogRead(photo_int_pin);
  int photo_ext = analogRead(photo_ext_pin) + PHOTO_OFFSET;
  if (photo_int > photo_ext) {
    //float dif;
    //dif = ((float)photo_int - (float)photo_ext) / 1023;
    //led_value16 -= ((int)((float)led_value16 * dif * .05))+1;
    led_value16 = max(led_value16 - 1, 0);
    //if ((led_value16 > 0) && (led_value16 < MIN_ILLUM16)) {led_value16 = MIN_ILLUM16;}
  } else if (photo_int < photo_ext) {
    //float dif;
    //dif = ((float)photo_ext - (float)photo_int) / 1023;
    //led_value16 += ((int)((float)led_value16 * dif *.05)+1);
    led_value16 = min(led_value16 + 1, 65535);
    if (led_value16 < MIN_ILLUM16) led_value16 = MIN_ILLUM16;
  }
  /*    else if (photo_int == photo_ext) {
    DEBUG_PRINT(DEBUG_VERBOSE, "Matched values! led_value16: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value16);
    DEBUG_PRINT(DEBUG_VERBOSE, " - photo_ext & photo_int: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, photo_int);
  }
  else {
    DEBUG_PRINT(DEBUG_ERROR, "Reached the else clause in photoresistor match mode, which should be impossible! led_value16: ");
    DEBUG_PRINT(DEBUG_ERROR, led_value16);
  }
  */    
  analogWrite16(LED_PIN16, led_value16);
  updateDisplay("Brightness Match", -1, led_value16, forceUpdate);

  // Write status
  if (currentMillis - previousMillis > 250 || forceUpdate == true) { // We don't want to print to serial/display at max speed, 4x per second should be fine.
    previousMillis = currentMillis;
    DEBUG_PRINT(DEBUG_VERBOSE, "---Status--- led_value16: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value16);
    DEBUG_PRINT(DEBUG_VERBOSE, ", photo_ext: ");
    DEBUG_PRINT(DEBUG_VERBOSE, photo_ext);
    DEBUG_PRINT(DEBUG_VERBOSE, ", photo_int: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, photo_int);
  }
  #endif
}

void handleDemoMode(bool forceUpdate) {
  #if defined(MODE_DEMO)
  if (forceUpdate == true) {
    demoDirectionUp = true;
    led_value = 0;
    led_value16 = 0;
    demoCounter = 255;
  }
  
  // Manage the LED intensity and update timing
  if (demoDirectionUp) {
    if (led_value16 < 65535 && currentMillis - previousMillis >= getNextDemoInterval(led_value16, true)) {
      previousMillis = currentMillis;
      led_value16++;
    } else if (led_value16 >= 65535) {
      demoDirectionUp = false;
      led_value16--;
    }
  } else {
    if (led_value16 > 0 && currentMillis - previousMillis >= getNextDemoInterval(led_value16, false)) {
      previousMillis = currentMillis;
      led_value16--;
    } else if (led_value16 <= 0) {
      demoDirectionUp = true;
      led_value16++;
    }
  }

  // Update PWM outputs to LED
  analogWrite(LED_PIN, led_value);
  analogWrite16(LED_PIN16, led_value16);
  updateDisplay("Demo Mode", -1, led_value16, forceUpdate);
  #endif
} // End Demo Mode

// Helper function to determine the next interval based on the current brightness and direction
unsigned long getNextDemoInterval(uint16_t led_value16, bool goingUp) {
    #if defined(MODE_DEMO)
    if (goingUp) {
    if (led_value16 < 60) return 100;
    else if (led_value16 < 255) return 40;
    else if (led_value16 < 511) return 20;
    else return 10;
  } else {
    if (led_value16 > 511) return 10;
    else if (led_value16 > 255) return 20;
    else if (led_value16 > 40) return 40;
    else return 100;
  }
  #endif
}

// Initialize EEPROM so we can retain last mode on resume
void initializeEEPROM() {
  uint8_t sig = EEPROM.read(ADDR_SIGNATURE);
  if (sig != EEPROM_SIGNATURE) {
    // Signature not matched, EEPROM not initialized
    DEBUG_PRINTLN(DEBUG_WARNING, "EEPROM Signature not matched, initializing the EEPROM addresses used by this sketch.");
    EEPROM.write(ADDR_SIGNATURE, EEPROM_SIGNATURE);
    EEPROM.write(ADDR_MODE, 0);  // Default mode
  }
}

// Code for 16-bit PWM - https://www.codrey.com/arduino-projects/arduino-advanced-16-bit-pwm/
void setupPWM16() {
  DDRB |= _BV(PB1) | _BV(PB2); //Set pins as outputs 
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) //Non-Inv PWM 
  | _BV(WGM11); // Mode 14: Fast PWM, TOP=ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12)
  | _BV(CS10); // Prescaler 1
  ICR1 = ICR; // TOP counter value (Relieving OCR1A*)
}

//* 16-bit version of analogWrite(). Only for D9 & D10
void analogWrite16(uint8_t pin, uint16_t val) {
  switch (pin) {
    case 9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}

// Read the momentary mode switch button
void readButton() {
  static unsigned long buttonPressedTime = 0; // Track when the button was initially pressed

  if (digitalRead(MODE_SW) == LOW) { // Button is pressed
    if (buttonPressedTime == 0) { // First detection of the button being pressed
      buttonPressedTime = currentMillis; // Record the time the button was pressed
        } else if ((currentMillis - buttonPressedTime > 4000)) { // Check for long press
      enterTimeSettingMode(); // Function to handle long press, enter time setting mode
            buttonPressedTime = 0; // Reset the timer after handling long press
            return; // Exit the function to avoid further processing in the same press
    }
  } else if (buttonPressedTime != 0) { // Button is not presed, but WAS pressed
    if ((currentMillis - buttonPressedTime > 25) && (currentMillis - buttonPressedTime <= 4000)) {
      // Handle normal button press if it was not a long press
      cycleModes(); // Function to cycle through modes or handle short press functionality
      previousMillis = 0; // Redundant, but I can't figure why this isn't being respected in cycleModes
    }
    // Reset variables for next button press
    buttonPressedTime = 0;
  }
}

void cycleModes() {
  previousMillis = 0; // Reset timing to ensure immediate mode update        

  // If the display is dimmed or off, restore full brightness and reset timers
  if (isDimmed || isOff) {
    lastInteractionMillis = currentMillis;
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(255);
    display.display();
    isDimmed = false;
    isOff = false;
  } else {
    // Cycle through modes, skipping undefined ones
    do {
      curmode = (curmode + 1) % 5;
      EEPROM.update(ADDR_MODE, curmode);  // Save new mode to EEPROM whenever it changes
      #ifndef MODE_POTENTIOMETER
      if (curmode == 0) continue;
      #endif
      #ifndef MODE_SKYLIGHT1
      if (curmode == 1) continue;
      #endif
      #ifndef MODE_SKYLIGHT2
      if (curmode == 2) continue;
      #endif
      #ifndef MODE_PHOTO_MATCH
      if (curmode == 3) continue;
      #endif
      #ifndef MODE_DEMO
      if (curmode == 4) continue;
      #endif
      break;
    } while (true);
    
    // New mode dependent actions
    if (curmode == 0) {
      DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 0 - Manual Dimming");
      handlePotentiometerMode(true);
    } else if (curmode == 1) {
      handleSkylightMode1(true);
      DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 1 - Skylight Mode 1");
    } else if (curmode == 2) {
      handleSkylightMode2(true);
      DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 2 - Skylight Mode 2");
    } else if (curmode == 3) {
      handlePhotoresistorMatchMode(true);
      DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 3 - Photo Match");
    } else if (curmode == 4) {
      DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 4 - Demo");
      #if defined(MODE_DEMO)      
      handleDemoMode(true);
      #endif
    } 
  } // End cycle through modes
} // End cycleModes definition

// When entering time setting mode, wait for user to set the potentiometer to approximate center before proceeding. 
void enterTimeSettingMode() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Set Time Mode");
  display.println("Adjust to middle to begin.");
  display.display();

  // Wait for user to adjust potentiometer to middle
  int potValue = 0;
  do {
    potValue = analogRead(POT_PIN);
    delay(100);
  } while (abs(potValue - 512) > 25); // Assume middle is around 512 in a 0-1023 range

  // Now adjust time
  adjustTimeWithPot();
}

// Adjust time in a few different speeds based on movement up or down from center
void adjustTimeWithPot() {
  display.clearDisplay();
  display.setCursor(0, 0);
  DateTime now = rtc.now(); // Assuming you have set up your RTC
  displayTimeSetting(now);

  while (digitalRead(MODE_SW) == HIGH) { // Exit loop when button is pressed again
    int potValue = analogRead(POT_PIN);
    int delta = potValue - 512;

    if (abs(delta) > 30 && abs(delta) < 60) {            // To avoid too sensitive adjustments
      now = now + TimeSpan(0, 0, 0, delta/5);               // 6-12 seconds per iteration
    } else if (abs(delta) >= 60 && abs(delta) < 240) {   // lets move a little quicker
      now = now + TimeSpan(0, 0, delta / 60, 0);            // 1-4 minutes per iteration
    } else if (abs(delta) >= 240 && abs(delta) < 480) {  // even quicker
      now = now + TimeSpan(0, 0, delta / 24, 0);            // 10-20 minutes per iteration
    } else if (abs(delta) >= 480) {                      // Fly - this is at the extremes of our analog read values - some POTs may not even be able to produce this value.
      now = now + TimeSpan(0, delta / 480, 0, 0);           // an hour per iteration
    }
    displayTimeSetting(now);
    delay(100);
  } // Button has been pressed again
  rtc.adjust(now); // Save the new time to RTC
  previousMillis = 0; // Reset previousMillis so whatever mode we are returning to updates the display.
}

// Function to display the current time in a specific format on the OLED
void displayTimeSetting(const DateTime &now) {
  char buffer[32]; // Enough to handle all characters
  int hour = now.hour();
  char am_pm[] = "AM";
  if (hour == 0) {
    hour = 12; // Midnight case
  } else if (hour == 12) {
    strcpy(am_pm, "PM"); // Noon case
  } else if (hour > 12) {
    hour -= 12;
    strcpy(am_pm, "PM");
  }

  snprintf(buffer, sizeof(buffer), "Set Time Mode\n%02d:%02d %s", hour, now.minute(), am_pm);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print(buffer);
  display.display();
}

// Updates to the OLED display
void updateDisplay(const char* mode, float elevation, uint16_t led_value16, bool forceUpdate) {
  // If the display is supposed to be off, or it has already been updated recently, skip updating the display
  if ((currentMillis - lastOledUpdateMillis) < OLED_UPDATE_INTERVAL || isOff) {
    if (forceUpdate == false) { // Unless force update was selected
      return;
    }
  }
  lastOledUpdateMillis = currentMillis; // Update the last update time

  // Start with the current mode on the first row
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(mode);
  display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines

  // Calculate brightness percentage from led_value16
  int percent = map(led_value16, 0, 65535, 0, 100);
  display.print("Brightness: ");
  display.print(percent);
  display.println("%");
  display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines

  // Display elevation in Skylight modes
  if (curmode == 1 || curmode == 2) {
    display.print("Ele: ");
    display.print(elevation, 1); // Print elevation with one decimal place
    display.write(248); // Degree symbol
    display.println();
    display.setCursor(0, display.getCursorY() + 1); // Little extra space between lines
  }
  
  // Add raw led_value16 in Demo mode
  if (curmode == 4) {
    display.print("Raw 16-bit: ");
    display.println(led_value16);
  }

  // Display current time bottom-right aligned
  DateTime now = rtc.now();
  char timeString[9]; // "HH:MM AM/PM"
  int hour = now.hour();
  String am_pm = "AM";
  if (hour == 0) {
      hour = 12;  // Midnight case
  } else if (hour == 12) {
      am_pm = "PM"; // Noon case
  } else if (hour > 12) {
      hour -= 12;
      am_pm = "PM";
  }
  sprintf(timeString, "%d:%02d %s", hour, now.minute(), am_pm.c_str());
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(timeString, 0, 0, &x1, &y1, &w, &h);  // Calculate the width and height of the time string
  display.setCursor(SCREEN_WIDTH - w - 1, SCREEN_HEIGHT - h); // Position the cursor for right alignment at the bottom
  display.print(timeString);
  
  // Done
  display.display();
}

// LED Gamma Correction Curve
uint8_t gammaCorrection(uint8_t led_value, float gamma) {
  // Normalize the input
  float normalized_input = (float)led_value / 255.0;

  // Apply gamma correction
  float corrected = pow(normalized_input, gamma);

  // Scale back to 0-255
  return (uint8_t)(255 * corrected);
}
uint16_t gammaCorrection16(uint16_t led_value16, float gamma) {
  // Normalize the input
  float normalized_input = (float)led_value16 / 65535.0;

  // Apply gamma correction
  float corrected = pow(normalized_input, gamma);

  // Scale back to 0-65535
  return (uint16_t)(65535 * corrected);
}
