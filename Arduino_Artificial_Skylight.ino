/* Code for TV panel (without the LCD) being used as a light.
So far we have 
- Solar elevation.
- Two different methods of mapping brightness based on elevation of sun.
- A LED brighness curve that really only works in 8-bit mode. Math doesn't seem to work as well with 16-bit PWM. 
- 16-bit PWM - MUCH smoother!
- Switch that can disable Skylight mode and read a potentiometer instead.

Designed for Board: Arduino Uno
*/

// #include <LiquidCrystal.h>
#include <SolarCalculator.h>
#include <RTClib.h>
#include <Wire.h>
#include <Math.h>
#include <Adafruit_SSD1306.h>
RTC_DS3231 rtc;

// Configuration Variables
const float latitude = 34.21316116274671; // Hardcode your current location for skylight mode
const float longitude = -84.54894616203148; // Hardcode your current location for skylight mode
const int time_zone = -4; // This just needs to match whatever time you programmed into your RTC device
const uint8_t daylight_value = 200; // Compared to full range of 0-255 - my light is way to bright for indoors 
const uint16_t daylight_value16 = 55000;
const uint8_t minIllum = 1;
const uint16_t minIllum16 = 25; // Because some displays really don't like being super low on the PWM signal.
const int photoOffset = -50; // The photoresistors aren't identical, and their connections and cord lengths aren't identical. The external reading will be adjusted by this much.
#define MODE_POTENTIOMETER // Only include the modes you want available
#define MODE_SKYLIGHT1
#define MODE_SKYLIGHT2
//#define MODE_PHOTO_MATCH
#define MODE_DEMO
unsigned long dimmingTime = 30000; // OLED dimming time in milliseconds (30 seconds)
uint8_t dimmedBrightness = 5; // OLED dimmed brightness as a percentage (5%)
unsigned long offTime = 300000; // OLED Off time in milliseconds (5 minutes)

// Pin Designations
const uint8_t led_pin = 11; 
const uint8_t led_pin16 = 9;
const uint8_t mode_sw = 8; 
const uint8_t photo_int_pin = A2;
const uint8_t photo_ext_pin = A1;
const uint8_t pot_pin = A0;

// OLED Setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Debugging setup
#define DEBUG_NONE 0
#define DEBUG_ERROR 1
#define DEBUG_WARNING 2
#define DEBUG_INFO 3
#define DEBUG_VERBOSE 4

#define DEBUG_LEVEL DEBUG_INFO // Set the debug level here

#define DEBUG_PRINT(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.print(message); } while (0)
#define DEBUG_PRINTLN(level, message) \
    do { if (DEBUG_LEVEL >= level) Serial.println(message); } while (0)


// Global Scope Declarations
unsigned long currentMillis = 0;        // stores the value of millis() in each iteration of loop()
unsigned long previousMillis = 0;       // for various timings
unsigned long previousButtonMillis = 0; // time when button press last occured - for debounce
uint8_t curmode = 0;                    // 0-Manual, 1-Skylight1, 2-Skylight2, 3-PhotoMatch, 4-Demo
int potval;                             // Used in mode 0 - value read from the potentiometer on pot_pin
int previousPotval = 0;                 // Used to change to manual mode from other modes
const int skyCheckInterval = 10000;     // Used in mode 1&2 - only update skylight every 10 seconds
int photo_int;                          // Used in mode 3 - value read from the indoor photoresistor
int photo_ext;                          // Used in mode 3 - value read from the outdoor photoresistor
boolean demoDirectionUp = true;         // Used in mode 4 - state variable
uint8_t demoCounter = 255;              // Used in mode 4 - increment the 8-bit PWM value once every 256 iterations of the 16-bit loop
uint8_t led_value = 10;                 // This will be the dynamic LED brighness sent to PWM
uint16_t led_value16 = minIllum16;      // 16-bit version. Only keeping the 8-bit for legacy compatibility
const uint16_t icr = 0xffff;            // I don't really understand this but it's used in setting up 16-bit PWM
unsigned long lastInteractionTime = 0;  // Global variable to track the last interaction time for OLED dimming
bool isDimmed = false;                  // Variable to track the current display brightness state
bool isOff = false;                     // Variable to track the current display brightness state

// Debounce variables for MODE_POTENTIOMETER
#if defined(MODE_POTENTIOMETER)
#define AVERAGE_BUFFER_SIZE 100         // Number of readings to average
int potReadings[AVERAGE_BUFFER_SIZE];   // Array to store the potentiometer readings
int readIndex = 0;                      // Index for the next reading
int total = 0;                          // Running total of the readings
int average = 0;                        // Average of the readings
#endif

// Variables used in Skylight Mode 1
#if defined(MODE_SKYLIGHT1)
float relative_brightness;
float relative_brightness16;
#endif

// Variables used in Skylight Mode 2
#if defined(MODE_SKYLIGHT2)
const float match_elevation = 1; // Elevation angle at which we switch between twilight and daylight models. Don't recommend you change it. 
const float astronomical_decay_constant = 0.15; // For how slow the twilight transformation goes
const float diffuse_scaling_factor = 1; // Will be multiplied. 1 means no change. Higher emphasises the impact of ambient reflection of light
float match_factor;
float max_combined_level = 0;
#endif

// Function Prototypes
void setupPWM16();
void analogWrite16(uint8_t pin, uint16_t val);
void readButton();
void updateDisplay(const char* mode, float elevation, uint16_t led_value16);
void writeDemo();
void handlePotentiometerMode();
void handleSkylightMode1();
void handleSkylightMode2();
void handlePhotoresistorMatchMode();
void handleDemoMode();
uint8_t gammaCorrection(uint8_t led_value, float gamma = 2.2);
uint16_t gammaCorrection16(uint16_t led_value16, float gamma = 2.2);

// *** SETUP ***
void setup() {
  Serial.begin(115200);
  Serial.println("Serial started");
  DEBUG_PRINTLN(DEBUG_INFO, "Setup started");

  pinMode(led_pin, OUTPUT);
  pinMode(mode_sw, INPUT_PULLUP);
  setupPWM16();
  
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
  
  // Debounce array init for MODE_POTENTIOMETER
  #if defined(MODE_POTENTIOMETER)
  for (int thisReading = 0; thisReading < AVERAGE_BUFFER_SIZE; thisReading++) {
    potReadings[thisReading] = 0;
  }
  #endif

  // Used in Skylight Mode 1 - Linear LED Brightness
  #if defined(MODE_SKYLIGHT1)
  relative_brightness = (255 * log10(2)) / log10(255); // https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
  relative_brightness16 = (65535 * log10(2)) / log10(65535);
  #endif

  // Used in Skylight Mode 2 - Calculate combined and twilight light intensities at a matching point
  #if defined(MODE_SKYLIGHT2)
  // Calculate the match factor - so we have a smooth transition between twilight and daylight modes
  match_factor = combined_daytime_brightness(match_elevation + 0.001) / astronomical_twilight_brightness(match_elevation - 0.001);
  
  // Calculate the max and min values of the ouput of our curve
  for (float elevation = -20; elevation <= 90; elevation += 1) {
      float combined_level = combined_brightness(elevation);
      if (combined_level > max_combined_level) {
          max_combined_level = combined_level;
      }
  }
  #endif

  // Track initial interaction time
  lastInteractionTime = millis();

  Serial.println("Ending Setup()");
  Serial.flush();
}

// *** MAIN LOOP ***
void loop() {  
  currentMillis = millis();   // capture the latest value of millis()
  potval = analogRead(pot_pin); // doing this now so I can use the value to check for mode change, and later if I'm in manual mode
  DEBUG_PRINT(DEBUG_VERBOSE, "Potval at start of loop: ");
  DEBUG_PRINT(DEBUG_VERBOSE, potval);
  DEBUG_PRINT(DEBUG_VERBOSE, ".  previousPotval: ");
  DEBUG_PRINTLN(DEBUG_VERBOSE, previousPotval);
  
  // Start by looking for a mode change
  readButton(); // looks for mode change switch use
  int potChange = abs(potval - previousPotval);
  if (potChange > 70) {
    curmode = 0;
    previousPotval = potval;
  }

  // Reset the last interaction time if there is any interaction
  if (potChange > 70 || digitalRead(mode_sw) == LOW) {
    lastInteractionTime = currentMillis;
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
  if ((currentMillis - lastInteractionTime >= dimmingTime) && !isDimmed) {
    uint8_t correctedDimmedBrightness = gammaCorrection(map(dimmedBrightness, 0, 100, 0, 255));
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(correctedDimmedBrightness);
    isDimmed = true;
  }

  // Off timer logic
  if ((currentMillis - lastInteractionTime >= offTime) && !isOff) {
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
void handlePotentiometerMode() {
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
  analogWrite(led_pin, led_value);
  analogWrite16(led_pin16, led_value16);

  // Update OLED content, if necessary, based on timing
  if (currentMillis - previousMillis > 250) { // Only update screen every quarter second
    previousMillis = currentMillis;
    updateDisplay("Manual Dimming", -1, led_value16);
  }
  #endif
}

// *** Artificial Skylight Mode 1 - Modeled
void handleSkylightMode1() {
  #if defined(MODE_SKYLIGHT1)
  if (currentMillis - previousMillis >= skyCheckInterval) {
    previousMillis = currentMillis;
    DEBUG_PRINTLN(DEBUG_INFO, "Mode 1");

    // Get the sun's current position
    double azimuth, elevation;
    DateTime now = rtc.now();
    calcHorizontalCoordinates(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), latitude, longitude, azimuth, elevation);

    DEBUG_PRINT(DEBUG_INFO, "Elevation value: ");
    DEBUG_PRINTLN(DEBUG_INFO, elevation);

    // Non-linear brightness growth of the sun based on elevation angle
    if (elevation >= 40) {
      led_value = daylight_value;
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = daylight_value16;
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 15) {
      led_value = map(elevation * 100, 15 * 100, 40 * 100, .85 * daylight_value, daylight_value);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, 15 * 100, 40 * 100, .85 * daylight_value16, daylight_value16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 3) {
      led_value = map(elevation * 100, 3 * 100, 15 * 100, .76 * daylight_value, .85 * daylight_value);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, 3 * 100, 15 * 100, .76 * daylight_value16, .85 * daylight_value16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= 0) {
      led_value = map(elevation * 100, 0 * 100, 3 * 100, .7 * daylight_value, .76 * daylight_value);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, 0 * 100, 3 * 100, .7 * daylight_value16, .76 * daylight_value16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
    } else if (elevation >= -3) {
      led_value = map(elevation * 100, -3 * 100, 0 * 100, .6 * daylight_value, .7 * daylight_value);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, -3 * 100, 0 * 100, .6 * daylight_value16, .7 * daylight_value16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
      led_value16 += map(elevation * 100, -3 * 100, 0 * 100, minIllum16, 0);
    } else if (elevation >= -12) {
      led_value = map(elevation * 100, -12 * 100, -3 * 100, 0, .6 * daylight_value);
      led_value = pow(2, (led_value / relative_brightness));
      led_value16 = map(elevation * 100, -12 * 100, -3 * 100, 0, .6 * daylight_value16);
      led_value16 = pow(2, (led_value16 / relative_brightness16));
      led_value16 += minIllum16;
    } else {
      led_value = 0;
      led_value16 = 0;
    }

    // Guard rails
    if (led_value > 0 && led_value < minIllum / 2) led_value = 0; // LED stays off until halfway to minIllum
    else if (led_value >= minIllum / 2 && led_value < minIllum) led_value = minIllum / 2; // LED turns on halfway
    if (led_value > daylight_value) led_value = daylight_value;
    if (led_value16 > 0 && led_value16 < minIllum16 / 2) led_value16 = 0; // LED stays off until halfway to minIllum16
    else if (led_value16 >= minIllum16 / 2 && led_value16 < minIllum16) led_value16 = minIllum16 / 2; // LED turns on halfway
    if (led_value16 > daylight_value16) led_value16 = daylight_value16;

    // Write it
    analogWrite(led_pin, led_value);
    analogWrite16(led_pin16, led_value16);

    updateDisplay("Skylight Mode 1", elevation, led_value16);

    DEBUG_PRINT(DEBUG_INFO, "LED Value: ");
    DEBUG_PRINT(DEBUG_INFO, led_value);
    DEBUG_PRINT(DEBUG_INFO, "   LED Value16: ");
    DEBUG_PRINTLN(DEBUG_INFO, led_value16);
  }
  #endif
}

// Artificial Skylight Mode 2
#if defined(MODE_SKYLIGHT2)
// Function to calculate the intensity of direct sunlight based on solar elevation using the Solar Air Mass model
float direct_sunlight_brightness(float elevation, float I0 = 1361, float k = 0.2) {
    if (elevation > match_elevation) {
        float zenith_angle = 90 - elevation;
        float AM = 1 / (cos(radians(zenith_angle)) + 0.50572 * pow((96.07995 - zenith_angle), -1.6364));
        return I0 * exp(-k * AM);
    } else {
        return 0;
    }
}

// Function to calculate diffuse sky light using the simplified Perez model
float diffuse_sky_brightness(float elevation, float a = -1, float b = -0.32, float c = 0.43, float d = 1.25, float e = -0.35) {
    if (elevation < match_elevation) {
        return 0;
    } else {
        float zenith_luminance = 100; // Assumed zenith luminance for a typical clear day
        return zenith_luminance * (1 + a * exp(b / (cos(radians(90 - elevation)) + 0.01)));
    }
}

// Function to calculate sky brightness during astronomical twilight using an exponential decay model
float astronomical_twilight_brightness(float elevation) {
    if (elevation > match_elevation) {
        return 0; // Daylight, no twilight effect
    } else {
        // Exponential decay of light intensity from sunset to end of astronomical twilight
        return exp(astronomical_decay_constant * elevation); // Adjust the decay rate to fit observational data
    }
}

// Function to scale and combine direct and diffuse daytime light components
float combined_daytime_brightness(float elevation) {
    float direct_component = direct_sunlight_brightness(elevation);
    float diffuse_component = diffuse_sky_brightness(elevation);
    return direct_component + (diffuse_component * diffuse_scaling_factor);
}

// Function to scale and combine the twilight and daylight components
float combined_brightness(float elevation) {
    float combined_daytime_level = combined_daytime_brightness(elevation);
    float astronomical_twilight_level = astronomical_twilight_brightness(elevation);
    return combined_daytime_level + (astronomical_twilight_level * match_factor); // Match factor is determined in the setup() function.
}
#endif

// And the main handler for this mode
void handleSkylightMode2() {
  #if defined(MODE_SKYLIGHT2)
  if (currentMillis - previousMillis >= skyCheckInterval) {
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
    if (led_value > 0 && led_value < minIllum / 2) led_value = 0; // LED stays off until halfway to minIllum
    else if (led_value >= minIllum / 2 && led_value < minIllum) led_value = minIllum / 2; // LED turns on halfway
    if (led_value > daylight_value) led_value = daylight_value;
    if (led_value16 > 0 && led_value16 < minIllum16 / 2) led_value16 = 0; // LED stays off until halfway to minIllum16
    else if (led_value16 >= minIllum16 / 2 && led_value16 < minIllum16) led_value16 = minIllum16 / 2; // LED turns on halfway
    if (led_value16 > daylight_value16) led_value16 = daylight_value16;
    
    // Write it
    analogWrite(led_pin, led_value);
    analogWrite16(led_pin16, led_value16);

    updateDisplay("Skylight Mode 2", elevation, led_value16);
  }
  #endif
}

// Photo Matching Mode
void handlePhotoresistorMatchMode() {
  #if defined(MODE_PHOTO_MATCH)
  int photo_int = analogRead(photo_int_pin);
  int photo_ext = analogRead(photo_ext_pin) + photoOffset;
  if (photo_int > photo_ext) {
    //float dif;
    //dif = ((float)photo_int - (float)photo_ext) / 1023;
    //led_value16 -= ((int)((float)led_value16 * dif * .05))+1;
    led_value16 = max(led_value16 - 1, 0);
    //if ((led_value16 > 0) && (led_value16 < minIllum16)) {led_value16 = minIllum16;}
  } else if (photo_int < photo_ext) {
    //float dif;
    //dif = ((float)photo_ext - (float)photo_int) / 1023;
    //led_value16 += ((int)((float)led_value16 * dif *.05)+1);
    led_value16 = min(led_value16 + 1, 65535);
    if (led_value16 < minIllum16) led_value16 = minIllum16;
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
  analogWrite16(led_pin16, led_value16);

  // Write status
  if (currentMillis - previousMillis > 250) { // We don't want to print to serial/display at max speed, 4x per second should be fine.
    previousMillis = currentMillis;
    DEBUG_PRINT(DEBUG_VERBOSE, "---Status--- led_value16: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value16);
    DEBUG_PRINT(DEBUG_VERBOSE, ", photo_ext: ");
    DEBUG_PRINT(DEBUG_VERBOSE, photo_ext);
    DEBUG_PRINT(DEBUG_VERBOSE, ", photo_int: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, photo_int);

    // Write OLED Content
    if (photo_ext > 999) photo_ext = 999; // Don't want to reserve another digit for the rare possibility of reaching max 1023, but need to protect from buffer overrun.
    if (photo_int > 999) photo_int = 999;
    updateDisplay("Brightness Match", -1, led_value16);
  }
  #endif
}

void handleDemoMode() {
  #if defined(MODE_DEMO)
  static unsigned long previousOledUpdateMillis = 0; // Static variable to keep track of OLED update timing
  uint16_t previous_led_value16 = led_value16;
  
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
  analogWrite(led_pin, led_value);
  analogWrite16(led_pin16, led_value16);

  // Reduce frequency of OLED updates to once every second
  if (currentMillis - previousOledUpdateMillis > 1000) { // 1000 ms = 1 second
    previousOledUpdateMillis = currentMillis;

    // Update OLED display
    updateDisplay("Demo Mode", -1, led_value16);

    // Debugging output
    DEBUG_PRINT(DEBUG_VERBOSE, "Demo led_value: ");
    DEBUG_PRINT(DEBUG_VERBOSE, led_value);
    DEBUG_PRINT(DEBUG_VERBOSE, " - led_value16: ");
    DEBUG_PRINTLN(DEBUG_VERBOSE, led_value16);
  }
  #endif
} // End Demo Mode

// Helper function to determine the next interval based on the current brightness and direction
unsigned long getNextDemoInterval(uint16_t led_value16, bool goingUp) {
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
}

// Code for 16-bit PWM - https://www.codrey.com/arduino-projects/arduino-advanced-16-bit-pwm/
void setupPWM16() {
  DDRB |= _BV(PB1) | _BV(PB2); //Set pins as outputs 
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) //Non-Inv PWM 
  | _BV(WGM11); // Mode 14: Fast PWM, TOP=ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12)
  | _BV(CS10); // Prescaler 1
  ICR1 = icr; // TOP counter value (Relieving OCR1A*)
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
  if (digitalRead(mode_sw) == LOW) {
    if ((millis() - previousButtonMillis) > 400) {
      previousButtonMillis = millis();
      
      // If the display is dimmed or off, restore full brightness and reset timers
      if (isDimmed || isOff) {
        lastInteractionTime = millis();
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(255);
        display.display();
        isDimmed = false;
        isOff = false;
      } else {
        // Cycle through modes, skipping undefined ones
        do {
          curmode = (curmode + 1) % 5;
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
        
        previousMillis = 0; // Make sure everything updates right away - reset counters after mode change
        
        if (curmode == 0) {
          DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 0 - Manual Dimming");
        } else if (curmode == 1) {
          DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 1 - Skylight Mode 1");
        } else if (curmode == 2) {
          DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 2 - Skylight Mode 2");
        } else if (curmode == 3) {
          DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 3 - Photo Match");
        } else if (curmode == 4) {
          DEBUG_PRINTLN(DEBUG_WARNING, "Entering mode 4 - Demo");
          demoDirectionUp = true;
          led_value = 0;
          led_value16 = 0;
          demoCounter = 256;
        }
      }
    } // End button held long enough to change
  } // End button currently pressed
} // End readButton definition

// Updates to the OLED display
void updateDisplay(const char* mode, float elevation, uint16_t led_value16) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(mode);
    display.setCursor(0, display.getCursorY() + 2); // Little extra space between lines

    // Always display elevation in Skylight modes
    if (strcmp(mode, "Skylight Mode 1") == 0 || strcmp(mode, "Skylight Mode 2") == 0) {
        display.print("Elevation: ");
        display.print(elevation, 1); // Print elevation with one decimal place
        display.write(248); // Degree symbol
        display.println();
        display.setCursor(0, display.getCursorY() + 2); // Little extra space between lines
    }

    // Calculate brightness percentage from led_value16
    int percent = map(led_value16, 0, 65535, 0, 100);
    display.print("Brightness: ");
    display.print(percent);
    display.println("%");
    display.setCursor(0, display.getCursorY() + 2); // Little extra space between lines
    
    // Add raw led_value16 in Demo mode
    if (strcmp(mode, "Demo Mode") == 0) {
        display.print("Raw 16-bit: ");
        display.println(led_value16);
    }
    
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
