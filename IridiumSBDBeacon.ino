// ####################################
// # Paul Clark's Iridium 9603 Beacon #
// ####################################

// Based on Mikal Hart's IridiumSBD Beacon example: https://github.com/mikalhart/IridiumSBD
// Requires Mikal's TinyGPS library: https://github.com/mikalhart/TinyGPS
// and PString: http://arduiniana.org/libraries/pstring/

// With grateful thanks to:
// Adafruit: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
// MartinL: https://forum.arduino.cc/index.php?topic=341054.msg2443086#msg2443086

// Hacked by Paul to run on the Adafruit Feather M0 (Adalogger)
// https://www.adafruit.com/products/2796
// GPS data provided by Adafruit Ultimate GPS Breakout or Ultimate GPS FeatherWing
// https://www.adafruit.com/products/746
// https://www.adafruit.com/products/3133
// (Don't fit the CR1220 back-up battery. VBAT is provided by the Adalogger 3V rail.)
// Pressure (altitude) and temperature provided by Sparkfun MPL3115A2 Breakout
// https://www.sparkfun.com/products/11084
// The Adafruit MPL3115A2 Breakout will work too
// https://www.adafruit.com/products/1893
// Requires Adafruit's MPL3115A2 library
// https://github.com/adafruit/Adafruit_MPL3115A2_Library

// Uses RTCZero to provide sleep functionality (on the M0)
// https://github.com/arduino-libraries/RTCZero

// With grateful thanks to CaveMoa for his SimpleSleepUSB example
// https://github.com/cavemoa/Feather-M0-Adalogger
// https://github.com/cavemoa/Feather-M0-Adalogger/tree/master/SimpleSleepUSB
// Note: you will need to close and re-open your serial monitor each time the M0 wakes up

// Iridium 9603 is interfaced to Adalogger using Serial2
// D6 (Port A Pin 20) = Enable (Sleep) : Connect to 9603 ON/OFF Pin 5
// D10 (Port A Pin 18) = Serial2 TX : Connect to 9603 Pin 6
// D12 (Port A Pin 19) = Serial2 RX : Connect to 9603 Pin 7
// A3 / D17 (Port A Pin 4) = Network Available : Connect to 9603 Pin 19
// Iridium 9603 is powered from Linear Technology LTC3225 SuperCapacitor Charger
// (fitted with 2 x 1F 2.7V caps e.g. Bussmann HV0810-2R7105-R)
// to provide the 1.3A peak current when the 9603 is transmitting
// http://www.linear.com/product/LTC3225
// http://www.linear.com/product/LTC3225#demoboards
// D5 (Port A Pin 15) = LTC3225 ~Shutdown
// A1 / D15 (Port B Pin 8) = LTC3225 PGOOD
// Connect LTC3225 VIN to Adalogger VBAT
// Connect LTC3225 GND to Adalogger GND
// Connect LTC3225 VOUT to Iridium 9603 EXT_PWR Pin 1 + 2
// Connect LTC3225 GND to Iridium 9603 EXT_GND Pin 3 + 4
// Connect 9603 RTS Pin 13 to SIG_GND Pin 18 (9603 won't communicate without this)
// Connect 9603 DTR Pin 14 to SIG_GND Pin 15 (9603 won't communicate without this)

// Ultimate GPS is interfaced to Adalogger using Serial1
// D1 (Port A Pin 10) = Serial1 TX : Connect to GPS RX
// D0 (Port A Pin 11) = Serial1 RX : Connect to GPS TX
// D11 (Port A Pin 16) = GPS ENable : Connect to GPS EN(ABLE)
// Connect Ultimate GPS VIN to Adalogger VBAT
// Connect Ultimate GPS GND to Adalogger GND
// Connect Ultimate GPS VBAT to Adalogger 3V (no need for a CR1220 backup battery)

// Adalogger has a built-in SD card slot
// D22 (Port A Pin 12) = MISO : Connected to SD Card
// D23 (Port B Pin 10) = MOSI : Connected to SD Card
// D24 (Port B Pin 11) = SCK : Connected to SD Card

// MPL3115A2 Pressure (Altitude) and Temperature Sensor
// D20 (Port A Pin 22) = SDA : Connect to MPL3115A2 SDA
// D21 (Port A Pin 23) = SCL : Connect to MPL3115A2 SCL
// Connect MPL3115A2 VCC/Vin to Ultimate GPS 3.3Vout (so GPS_EN can disable both GPS and MPL3115A2 to save power)
// Connect MPL3115A2 GND to Adalogger GND

// More Adalogger pins
// D13 (Port A Pin 17) = Red LED
// D9 (Port A Pin 7) = AIN 7 : Battery Voltage / 2
// D8 (Port A Pin 6) = Green LED
// D16 / A2 (Port B Pin 9) : Reserve for (e.g.) DS3231 RTC Featherwing Interrupt

// If you are using the LTC3225 demo circuit (1220B):
// Set VOUT_SELECT to 4.8V (Connect VSEL to GND)
// Set ICHRG to 150mA (Connect PROG to GND via 15k in parallel with 60.4k)
// Remove the Run/SHDN link (the Adalogger controls the SHDN pin)
// Remove the 0.25F low profile capacitor from the rear of the PCB
// Fit two (e.g.) Bussmann HV0810-2R7105-R 1F 2.7V caps in OPT C3 and OPT C4
// There is no thermal relief on the OPT C4 GND pin and it can be difficult to solder
// You might have to link the cap leg to the low profile cap negative pad instead

#include <IridiumSBD.h>
#include <TinyGPS.h> // NMEA parsing: http://arduiniana.org
#include <PString.h> // String buffer formatting: http://arduiniana.org

#include <RTCZero.h> // M0 Real Time Clock
RTCZero rtc; // Create an rtc object
int BEACON_INTERVAL = 10; // Define how often messages are sent in MINUTES (suggested values: 10,12,15,20,30,60,120,180,240)
// BEACON_INTERVAL can be modified during code execution e.g. when iterationCounter reaches a value [Line 284-285]

// MPL3115A2
#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2 (SC1PAD2)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3 (SC1PAD3)
// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
HardwareSerial &ssIridium(Serial2);

#define ssGPS Serial1 // Use Adalogger (Cortex M0) Serial1 to interface to the Ultimate GPS

IridiumSBD isbd(ssIridium, 6); // Iridium Sleep connected to D6
TinyGPS tinygps;
static const int ledPin = 13; // Red LED on pin D13
long iterationCounter = 0;

static const int greenLED = 8; // Green LED on pin D8
static const int networkAvailable = 17; // 9602 Network Available on pin D17
static const int LTC3225shutdown = 5; // LTC3225 ~Shutdown on pin D5
static const int LTC3225PGOOD = 15; // LTC3225 PGOOD on pin A1 / D15
static const int GPS_EN = 11; // Ultimate GPS Enable on pin D11

// IridiumSBD Callback
bool ISBDCallback()
{
  digitalWrite(ledPin, (millis() / 1000) % 2 == 1 ? HIGH : LOW);
  return true;
}

// Interrupt handler for SERCOM1 (essential for Serial2 comms)
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

// RTC alarm interrupt
void alarmMatch()
{
  int rtc_mins = (rtc.getMinutes() + BEACON_INTERVAL) % 60; // Read the RTC minutes; Increase by BEACON_INTERVAL; Compensate for 60 minute roll over
  rtc.setAlarmMinutes(rtc_mins); // Set next alarm time (minutes)
  rtc.enableAlarm(rtc.MATCH_MMSS); // Alarm Match on minutes and seconds
  if (BEACON_INTERVAL > 60) { // If BEACON_INTERVAL is greater than an hour
    // Read the RTC hours; Increase by the number of whole hours in BEACON_INTERVAL; Compensate for 24 hour roll over
    int rtc_hours = (rtc.getHours() + (BEACON_INTERVAL / 60)) % 24;
    rtc.setAlarmHours(rtc_hours); // Set next alarm time (hours)
    rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
  }
}

void setup()
{
  rtc.begin(); // Start the RTC
  rtc.setAlarmSeconds(rtc.getSeconds()); // Initialise RTC Alarm Seconds
  alarmMatch(); // Set next alarm time
  rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
  
  pinMode(ledPin, OUTPUT); // Adalogger Red LED

  pinMode(greenLED, OUTPUT); // Adalogger Green LED
  
  pinMode(LTC3225shutdown, OUTPUT); // LTC3225 supercapacitor charger shutdown pin
  digitalWrite(LTC3225shutdown, HIGH); // Enable the LTC3225 supercapacitor charger
  pinMode(LTC3225PGOOD, INPUT); // Define an input for the LTC3225 Power Good signal
  
  pinMode(GPS_EN, OUTPUT); // Adafruit Ultimate GPS Regulator enable
  digitalWrite(GPS_EN, HIGH); // Enable the Ultimate GPS
  
  pinMode(networkAvailable, INPUT); // Define an input for the Iridium 9603 Network Available signal
  
  // Start the serial console
  Serial.begin(115200);

  // flash red and green LEDs on reset
  for (int i=0; i <= 4; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    digitalWrite(greenLED, HIGH);
    delay(200);
    digitalWrite(greenLED, LOW);
  }
  delay(18000); // Wait remainder of 20secs - allow time for super caps to charge and for user to open serial monitor

  // Send welcome message
  Serial.println("IridiumSBDBeacon");
  // Check LTC3225 PGOOD
  Serial.println("Checking LTC3225 PGOOD...");
  int PGOOD = digitalRead(LTC3225PGOOD);
  while (PGOOD == LOW) {
    Serial.println("Waiting for PGOOD to go HIGH...");
    delay(1000);
    PGOOD = digitalRead(LTC3225PGOOD);
  }
  Serial.println("LTC3225 PGOOD OK!");

  // Setup the IridiumSBD
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  isbd.useMSSTMWorkaround(false);
}

void loop()
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long dateFix, locationFix;
  float latitude, longitude;
  long altitude;
  bool fixFound = false;
  bool charsSeen = false;
  unsigned long loopStartTime = millis();

  // Step 0: Start the serial ports
  ssIridium.begin(19200);
  ssGPS.begin(9600);

  // Configure Ultimate GPS
  Serial.println("Configuring GPS...");
  ssGPS.println("$PMTK220,1000*1F"); // Set NMEA Update Rate to 1Hz
  delay(100);
  ssGPS.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); // Set NMEA Output to GGA and RMC
  delay(100);
  ssGPS.println("$PGCMD,33,0*6D"); // Disable Antenna Updates
  delay(1100); // Delay for > 1 second

  // Step 1: Reset TinyGPS and begin listening to the GPS
  Serial.println("Beginning to listen for GPS traffic...");
  tinygps = TinyGPS();
  
  // Step 2: Look for GPS signal for up to 7 minutes
  for (unsigned long now = millis(); !fixFound && millis() - now < 7UL * 60UL * 1000UL;)
  {
    if (ssGPS.available())
    {
      charsSeen = true;
      if (tinygps.encode(ssGPS.read()))
      {
        tinygps.f_get_position(&latitude, &longitude, &locationFix);
        tinygps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &dateFix);
        altitude = tinygps.altitude();
        fixFound = locationFix != TinyGPS::GPS_INVALID_FIX_TIME && 
                   dateFix != TinyGPS::GPS_INVALID_FIX_TIME && 
                   altitude != TinyGPS::GPS_INVALID_ALTITUDE &&
                   year != 2000;
      }
    }
    ISBDCallback(); // We can call it during our GPS loop too.

    // if we haven't seen any GPS in 5 seconds, then the wiring is wrong.
    if (!charsSeen && millis() - now > 5000)
    break;
  }

  Serial.println(charsSeen ? fixFound ? F("A GPS fix was found!") : F("No GPS fix was found.") : F("Wiring error: No GPS data seen."));

  // Start the MPL3115A2 (does Wire.begin())
  float pascals, tempC;
  if (baro.begin()) {
    pascals = baro.getPressure();
    if (pascals > 110000) pascals = 0.0; // Correct wrap-around if pressure drops too low
    tempC = baro.getTemperature();
  }
  else {
    Serial.println("Error initialising MPL3115A2!");
    pascals = 0.0;
    tempC = 0.0;
  }

  // Read battery voltage
  float vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
  // check if voltage is >= 3.55V
  if (vbat < 3.55) {
    Serial.println("!!LOW BATTERY!!");
  }

  // Step 3: Start talking to the 9603 and power it up
  Serial.println("Beginning to talk to the 9603...");
  
  ++iterationCounter; // Increment iterationCounter

  // Update BEACON_INTERVAL if required (comment these lines out if you want BEACON_INTERVAL to remain constant)
  // Remember BEACON_INTERVAL is an int, so don't get too carried away!
  //if (iterationCounter > 12) BEACON_INTERVAL = 60; // Send every 10 mins for the first two hours then drop to once per hour
  //if (iterationCounter > 250) BEACON_INTERVAL = 180; // After ten days, drop to once every three hours
  
  if (isbd.begin() == ISBD_SUCCESS)
  {
    char outBuffer[80]; // Always try to keep message short

    if (fixFound)
    {
      sprintf(outBuffer, "%d%02d%02d%02d%02d%02d,", year, month, day, hour, minute, second);
      int len = strlen(outBuffer);
      PString str(outBuffer + len, sizeof(outBuffer) - len);
      str.print(latitude, 6);
      str.print(",");
      str.print(longitude, 6);
      str.print(",");
      str.print(altitude / 100);
      str.print(",");
      str.print(tinygps.f_speed_knots(), 1);
      str.print(",");
      str.print(tinygps.course() / 100);
      str.print(",");
      str.print(pascals, 0);
      str.print(",");
      str.print(tempC, 1);
      str.print(",");
      str.print(vbat, 2);
    }

    else
    {
      sprintf(outBuffer, "%d: No GPS fix found!", iterationCounter);
    }

    Serial.print("Transmitting message '");
    Serial.print(outBuffer);
    Serial.println("'");
    isbd.sendSBDText(outBuffer);

    Serial.println("Putting 9603 in sleep mode...");
    isbd.sleep();
  }

  // Get ready for sleep
  Serial.println("Going to sleep until next alarm time...");
  ssIridium.end();
  ssGPS.end();
  delay(1000); // Wait for serial ports to clear

  // Disable LEDs
  digitalWrite(ledPin, LOW);
  digitalWrite(greenLED, LOW);
  pinMode(ledPin, INPUT);
  pinMode(greenLED, INPUT);

  // Save power by disabling both GPS and Iridium supercapacitor charger
  Wire.end(); // Stop I2C comms
  digitalWrite(GPS_EN, LOW); // Disable the Ultimate GPS and MPL3115A2
  digitalWrite(LTC3225shutdown, LOW); // Disable the LTC3225 supercapacitor charger

  // Close and detach the serial console (as per CaveMoa's SimpleSleepUSB)
  Serial.end(); // Close the serial console
  USBDevice.detach(); // Safely detach the USB prior to sleeping

  // Sleep until next alarm match
  rtc.standbyMode();

  // Check battery voltage after sleep
  // If voltage is <3.55V, everything is still powered down so go back to sleep and wait for battery to recharge
  delay(1100); // Let things stabilise and make sure rtc moves on by at least 1 second (redundant?)
  vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0); // Read battery voltage
  while (vbat < 3.55) { // Is voltage <3.55V?
    rtc.standbyMode(); // Sleep again
    delay(1100); // Let things stabilise again
    vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0); // Read battery voltage again
  }

  // Attach and reopen the serial console
  USBDevice.attach(); // Re-attach the USB
  delay(1000);  // Delay added to make serial more reliable
  Serial.begin(115200); // Restart serial console
  Serial.println("Wake up!");

  // Enable LEDs
  pinMode(ledPin, OUTPUT);
  pinMode(greenLED, OUTPUT);

  // Re-enable GPS and Iridium supercapacitor charger
  digitalWrite(GPS_EN, HIGH); // Enable the Ultimate GPS and MPL3115A2
  digitalWrite(LTC3225shutdown, HIGH); // Enable the LTC3225 supercapacitor charger

  // flash red and green LEDs on wake
  for (int i=0; i <= 4; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    digitalWrite(greenLED, HIGH);
    delay(200);
    digitalWrite(greenLED, LOW);
  }
  delay(18000); // Wait remainder of 20secs - allow time for super caps to recharge

  // Check LTC3225 PGOOD
  Serial.println("Checking LTC3225 PGOOD...");
  int PGOOD = digitalRead(LTC3225PGOOD);
  while (PGOOD == LOW) {
    Serial.println("Waiting for PGOOD to go HIGH...");
    delay(1000);
    PGOOD = digitalRead(LTC3225PGOOD);
  }
  Serial.println("LTC3225 PGOOD OK!");
}


