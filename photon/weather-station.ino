// The included MQTT library only allows a max packet size of 255.  According to the library, the packet
// size is the total of [MQTT Header(Max:5byte) + Topic Name Length + Topic Name + Message ID(QoS1|2) + Payload]
// The max packet size can be overwritten with the MQTT_MAX_PACKET_SIZE define.

#include <MQTT.h>
#include <SparkFun_Photon_Weather_Shield_Library.h>
#include <SparkJson.h>

// Enable system threading so application loop is not blocked by background tasks, networking, or 
// Particle cloud tasks.  Without threading, if the WiFi connection or internet connection went down,
// then the application loop would be blocked for up to 30-40 seconds at a time.  With system threading
// enabled the application setup() and loop() functions are called immediately without waiting for a
// WiFi or Cloud connection.  However we will wait for a WiFi connection in setup().
SYSTEM_THREAD(ENABLED);

// Defines for enabling Serial and Particle Cloud debug messages
#define DEBUG_SERIAL
#define DEBUG_CLOUD

// Defines for extra level of debug
#define DEBUG_WIND_SPEED
// #define DEBUG_WIND_DIR
#define DEBUG_RAIN

// Defines for pins
#define BUILTIN_LED     D7  // Built in LED pin
#define WIND_DIR        A0  // Wind direction sensor
#define WIND_SPEED      D3  // Wind speed sensor
#define RAIN            D2  // Rain sensor

#ifdef DEBUG_WIND_SPEED
#define WIND_SPEED_TRIG D6  // Debug output for wind speed ISR trigger
#endif

#ifdef DEBUG_RAIN
#define RAIN_TRIG       D5  // Debug output for rain ISR trigger (same as unused SOIL_MOIST_POWER pin)
#endif

// Interval at which to read the sensors in milliseconds
#define READ_INTERVAL 10000

// Degree offset of wind direction sensor due to mounting orientation
#define WIND_DIR_OFFSET 90.0

// Size of circular buffer for WiFi signal strength
#define SIG_STRENGTH_BUF_SIZE 100

#define SIG_STRENGTH_LIMIT -80

enum State {SETUP, INIT, NO_WIFI, WEAK_SIG, NO_MQTT, NO_CLOUD, GOOD, READ_SENSORS, UPDATE_RAIN, EXIT_APP, JSON_ERR};

// Define a byte array for the broker IP address
uint8_t brokerIP[] = {192, 168, 20, 4};

// The topic to publish sensor data on.  The + is to be replaced by the device ID.
String mqttSensorOutputTopic = "sensors/+/data";

// The topic to subscribe to for control.  The # is to be replaced by the device ID.
String mqttControlInputTopic = "sensors/+/control";

// The topic to publish debug data on.  The + is to be replaced by the device ID.
String mqttDebugOutputTopic = "sensors/+/debug";

// Define some globals
String deviceId;
String hostName;
String mqttClientId;

// Pointer to MQTT client.  Actual object is created with new once broker info is known.
MQTT* mqttClient;

// Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barometric sensor
Weather sensor;

// Global variables modified by ISRs must be volatiles
volatile unsigned int windTicks = 0;
volatile unsigned int rainAccum = 0;

#ifdef DEBUG_WIND_SPEED
volatile unsigned int windRaw = 0;
volatile unsigned int windDebounced = 0;
#endif

#ifdef DEBUG_RAIN
volatile unsigned int rainRaw = 0;
volatile unsigned int rainDebounced = 0;
#endif

// Global variables and arrays for storing rainfall history
float rainLastHr;
float rainLast24Hr;
float rainDaily[7];
unsigned int rainAccumMinute[60];
unsigned int rainAccumHour[24];
unsigned int rainAccumDay[7];

// Function prototypes
void check_wifi(void);
void setup_mqtt_broker(void);
void check_network(State* currentState);
void mqtt_reconnect(int sigStrength);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void wind_speed_isr(void);
void rain_isr(void);
void get_weather(State currentState);
float get_wind_speed(void);
float get_wind_direction(void);
void get_rainfall(State currentState);
void set_led_color(State color);
void update_time(void);
void debug_message(String msg, bool cloud=false);

// Setup routine.  Run once at bootup.
void setup() {
    
#ifdef DEBUG_SERIAL

    unsigned long startTime;
    
    Serial.begin(9600);
    
    // Wait for the host to open the serial port.  Timemout and continue if the
    // host never connects to prevent the application from hanging.  If the USB
    // is connected to a PC this should automatically happen regardless if a terminal
    // emulator window is open.  I think it just means the USB port is connected.
    // The USB port always seems to disconnect and reconnect on the Photon after
    // code is flashed or a reset occurs.
    startTime = millis();
    
    while ((millis() - startTime < 5000) && (! Serial.isConnected()))
        // Need to call this to process Cloud data, since we are stuck in a long loop.
        // The cloud process() routine is called after every loop() and during
        // delays.  It must be called if the loop() is blocked by code execution.
        // This is only true if system threading is not enabled and not really needed
        // if system threading is enabled.
        Particle.process();
    
    // The host has connected.  Wait for user to open terminal emulator window,
    // select the proper port, and send a CTRL-Z character to know that the user
    // terminal emulator window is ready to receive messages.  If user never connects
    // and sends a proper character, timeout after 30 seconds so bootup will continue.
    if (Serial.isConnected()) {
        
        startTime = millis();
        
        // The host USB port seems to send a sequence of characters every so often.
        // I could not use any character because this sequence would cause the wait
        // loop to drop out.  Therefore we will look specifically for a CTRL-Z which
        // is ASCII character 26 decimal.  If no character is in the buffer then
        // the Serial.read() returns -1.
        while ((millis() - startTime < 30000) && (Serial.read() != 26))
            Particle.process();
        
    }
    
#endif

    debug_message("");
    debug_message("Running setup()", true);
    debug_message("");
    
    // Take over control of the RGB LED.  Prior to this the LED should be 
    // breathing cyan.
    RGB.control(true);
    
    // Set the LED color to indicate we are in setup().  It will stay this
    // color until we connect to WiFi and mDNS finds the MQTT broker.
    // Delay to make sure we can see the LED color before exiting setup.
    set_led_color(SETUP);
    delay(1000);

    // Initialize the BUILTIN_LED pin as an output
    pinMode(BUILTIN_LED, OUTPUT);

    // Initialize the wind speed and rain sensor input pins.  These devices have simple
    // reed switch contacts and the SparkFun shield does not have built in pullups, so
    // the internal pullups of the Photon need to be enabled.
    pinMode(WIND_SPEED, INPUT_PULLUP);
    pinMode(RAIN, INPUT_PULLUP);

    // Debug ISR trigger outputs
#ifdef DEBUG_WIND_SPEED
    pinMode(WIND_SPEED_TRIG, OUTPUT);
#endif

#ifdef DEBUG_RAIN
    pinMode(RAIN_TRIG, OUTPUT);
#endif

    // Set the timezone to Mountain Time (UTC - 7)
    Time.zone(-7);

    // Update the time to get the Daylight Savings Time setting correct
    update_time();
    
    debug_message("Date/Time: " + Time.timeStr() + " DST is " + (Time.isDST() ? "ON" : "OFF"));
    
    debug_message("Firmware Rev: " + System.version(), true);

    deviceId = System.deviceID();
    debug_message("Device ID: " + deviceId);

    // Modify the topics now that we know the device ID.
    mqttSensorOutputTopic.replace("+", deviceId);
    mqttControlInputTopic.replace("+", deviceId);
    mqttDebugOutputTopic.replace("+", deviceId);

    // Create a host name for the device
    hostName = "photon_" + deviceId;
    debug_message("Hostname: " + hostName);
  
    // Create an unique MQTT client ID.  We are creating the unique ID from the
    // the deivice ID which is quite long.  The MQTT 3.1 spec says this must be between
    // 1 and 23 characters.  This requirement may have been removed in the 3.1.1 spec.
    // However I think that mosquitto will support longer client ID lengths anyway, so
    // we should be OK.
    mqttClientId = "photon_" + deviceId;
    debug_message("MQTT Client ID: " + mqttClientId);
    debug_message("");

    // Initialize the I2C sensors
    debug_message("Starting I2C sensors");
    
    // This will print out which devices it has detected
    sensor.begin();
    debug_message("");

    // You can only receive accurate barometric readings or accurate altitiude
    // readings at a given time, not both at the same time. The following two lines
    // tell the sensor what mode to use. You could easily write a function that
    // takes a reading in one mode and then switches to the other mode to grab that
    // reading, resulting in data that contains both accurate altitude and barometric
    // readings. For this weather station, we will only be using the barometer mode.
    // Be sure to only uncomment one line at a time.
    sensor.setModeBarometer();
    // sensor.setModeAltimeter();

    // These are additional MPL3115A2 functions the must be called for the sensor to work.
    
    // Set the oversample rate.  Call with a rate from 0 to 7. See page 33 of the
    // MPL3115A2 data sheet for a table of ratios.  The oversample range is 1 to 128.
    // The greater the oversample rate the lower the noise of the measurement, but
    // increases the time between when data samples can be taken.  A setting of 7 equals
    // an oversample rate of 128, with a minimum sample rate of 512 msec
    sensor.setOversampleRate(7);

    // Necessary call to enable temp, baro, and alt readings 
    sensor.enableEventFlags();

    // Check the WiFi connection
    check_wifi();

    // Setup the connection to the MQTT broker
    setup_mqtt_broker();
    
    // Initialize the arrays to zero
    int i;
    
    for (i = 0; i < 7; i++)
        rainDaily[i] = 0.0;
    
    for (i = 0; i < 60; i++)
        rainAccumMinute[i] = 0;
        
    for (i = 0; i < 24; i++)
        rainAccumHour[i] = 0;

    for (i = 0; i < 7; i++)
        rainAccumDay[i] = 0;
    
    // Attach external IRQ pins to ISR functions
    attachInterrupt(WIND_SPEED, wind_speed_isr, FALLING);
    attachInterrupt(RAIN, rain_isr, FALLING);

    // Turn on interrupts
    interrupts();

    debug_message("Finished setup()", true);
    debug_message("");
    
    // Turn off the LED when exiting setup
    set_led_color(EXIT_APP);

}

// Main loop.  Repeatedly called by system firmware.
void loop() {

    static State currentState = INIT;

    // Restore the LED color from the previous application loop exit
    set_led_color(currentState);
    
    // Check the state of the network connection.  The currentState can be
    // modified upon return
    check_network(&currentState);

    // Update the LED color
    set_led_color(currentState);

    // Have the MQTT client process any data.
    mqttClient->loop();

    // Restore the LED color
    set_led_color(currentState);
    
    // Update the current time.  Checks for Daylight Savings Time switch and
    // resyncs time to cloud once per day
    update_time();
    
    // Get the weather sensor readings
    get_weather(currentState);    

    // Restore the LED color
    set_led_color(currentState);
    
    // Update the rainfall history
    get_rainfall(currentState);

    // Restore the LED color
    set_led_color(currentState);
    
    // Turn LED off when exiting loop.  This will tell us if the system is blocking
    // the application loop.  With system threading enabled this should no longer
    // be an issue.
    set_led_color(EXIT_APP);
    
}

// Check the WiFi connection.
void check_wifi() {

    byte mac[6];
    
    debug_message("Checking the WiFi connection...");
    
    while (! WiFi.ready()) {
        
        // Pulse the LED indicate we have not successfully connected to WiFi yet    
        set_led_color(NO_WIFI);
        delay(500);
        set_led_color(SETUP);
        
        debug_message("WiFi not connected...checking again in 10 seconds");
        delay(10000);
        
    }

    debug_message("WiFi connected");
    debug_message(String::format("Access Point SSID:  %s", WiFi.SSID()));
    WiFi.BSSID(mac);
    debug_message(String::format("AP MAC address:     %02X:%02X:%02X:%02X:%02X:%02X",  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
    debug_message(String::format("Device IP address:  %s", WiFi.localIP().toString().c_str()));
    WiFi.macAddress(mac);
    debug_message(String::format("Device MAC address: %02X:%02X:%02X:%02X:%02X:%02X",  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]));
    debug_message("");
  
}

// Use mDNS to locate an MQTT broker and create the client instance.
// The actual connect will happen in the loop()
void setup_mqtt_broker() {
    
/*
    if (!MDNS.begin(hostName)) {4}

    // Scan until an MQTT broker is found
    Serial.println("Using mDNS to find an MQTT broker");
    while (true) {
        MDNS.update();
        int n = MDNS.queryService("mqtt", "tcp");
        if (n == 0) {
            Serial.println("No MQTT services found...retry in 60 seconds");
            // Wait 60 seconds before retrying
            delay(60000);
        }
        else {
            Serial.println("Found an MQTT broker");
            Serial.print("Hostname:   ");
            Serial.println(MDNS.hostname(0));
            Serial.print("IP address: ");
            Serial.println(MDNS.IP(0));
            Serial.print("Port num:   ");
            Serial.println(MDNS.port(0));
            Serial.println("");
*/    
            // Set the MQTT server and port to attach to
            
            // mDNS not implemented yet so hard code the address and port

            // brokerIP is a pointer to an array of bytes and must be persistent once this
            // routine exits, meaning use new or make it a global array.
            mqttClient = new MQTT(brokerIP, 1883, mqtt_callback);
            
/*
            break;
        }
    }
*/
}

// Check the network connections
void check_network(State* currentState) {
    
    static int sigStrengthBuffer[SIG_STRENGTH_BUF_SIZE];
    static int bufPtr = 0;
    static unsigned long lastDebugMessage = 0;
    
    // Initialize the signal strength buffer to zero if in INIT state or the
    // WiFi connection is down.
    if (*currentState == INIT || *currentState == NO_WIFI)
        for (int i = 0; i < SIG_STRENGTH_BUF_SIZE; i++)
            sigStrengthBuffer[i] = 0;
    
    // Get the current WiFi signal strength and put it in the circular buffer
    sigStrengthBuffer[bufPtr] = WiFi.RSSI();
    bufPtr = (bufPtr + 1) % SIG_STRENGTH_BUF_SIZE;

    unsigned long currentTime = millis();

    // The WiFi connection is down
    if (! WiFi.ready()) {
        
        set_led_color(NO_WIFI);
        
        // If we just got to this state or it has been 60 secs since the last message,
        // then print the debug message
        if ((*currentState != NO_WIFI) || (currentTime - lastDebugMessage > 60000)) {
            debug_message("No WiFi connection");
            lastDebugMessage = currentTime;
        }
        
        // Update the current state
        *currentState = NO_WIFI;
        
    }
    
    // WiFi is connected but we are not connected to the MQTT broker
    else if (! mqttClient->isConnected()) {
        
        // If we got to this state from a connected state then print the debug message
        // that we lost the MQTT broker connection
        if (*currentState == GOOD || *currentState == NO_CLOUD)
            debug_message ("Lost connection to MQTT broker");
        
        int i;
        int avgSigStrength = 0;
        
        // Determine if the signal strength history is good and calculate an average signal strength
        for (i = 0; i < SIG_STRENGTH_BUF_SIZE; i++) {
            avgSigStrength += sigStrengthBuffer[i] * 10;
            if (sigStrengthBuffer[(bufPtr - 1 - i + SIG_STRENGTH_BUF_SIZE) % SIG_STRENGTH_BUF_SIZE] < SIG_STRENGTH_LIMIT ||
                sigStrengthBuffer[(bufPtr - 1 - i + SIG_STRENGTH_BUF_SIZE) % SIG_STRENGTH_BUF_SIZE] >= 0) {
                break;
            }
        }
        
        // If the above for loop did not get through the entire buffer then we had a bad signal strength value
        if (i < SIG_STRENGTH_BUF_SIZE) {
            
            // Do not attempt to connect to MQTT broker.  WiFi signal error or signal too weak.
            set_led_color(WEAK_SIG);
            
            int failure = sigStrengthBuffer[(bufPtr - 1 - i + SIG_STRENGTH_BUF_SIZE) % SIG_STRENGTH_BUF_SIZE];
            
            // If we just got to this state or it has been 10 secs since the last message,
            // then print the debug message
            if (*currentState != WEAK_SIG || (currentTime - lastDebugMessage > 10000)) {
                if (failure < 0)
                    debug_message(String::format("WiFi signal too weak to attemmpt MQTT broker connection: %d dBm", failure));
                else
                    debug_message(String::format("Cannot attempt MQTT broker connection because of WiFi signal strength error: %d", failure));
                
                lastDebugMessage = currentTime;
            }
            
            // Update the current state
            *currentState = WEAK_SIG;
            
        }
        
        // No errors or signal strength values beyond the limit
        else {
            
            // Not connected to MQTT broker, set the LED and try to reconnect.
            set_led_color(NO_MQTT);
            
            // Calculate the average signal strength
            avgSigStrength /= SIG_STRENGTH_BUF_SIZE;
            avgSigStrength += 5;
            avgSigStrength /= 10;
            
            // Update the current state
            *currentState = NO_MQTT;
            
            // Attempt to reconnect
            mqtt_reconnect(avgSigStrength);
            
        }
    }
    
    // WiFi connected and MQTT broker connected but no Particle Cloud connection
    else if (! Particle.connected()) {
        
        set_led_color(NO_CLOUD);
        
        // If we just got to this state or it has been 10 secs since the last message,
        // then print the debug message
        if (*currentState != NO_CLOUD || (currentTime - lastDebugMessage > 10000)) {
            debug_message("No Particle Cloud connection");
            lastDebugMessage = currentTime;
        }
            
        // Update the current state
        *currentState = NO_CLOUD;
        
    }
    
    // All the network connections are up
    else {
        
        set_led_color(GOOD);
        
        if (*currentState != GOOD)
            debug_message("All Network connections restored");
            
        *currentState = GOOD;
        
    }

}

// Connect to the MQTT broker
void mqtt_reconnect(int sigStrength) {

    static unsigned long lastAttempt = 0;
    unsigned long currentTime = millis();
    
    // If not connected and the retry timeout has expired then retry again.  Do not block
    // here as we want the rainfall history to go ahead and update even though we cannot
    // send data while the broker connection is down.
    if (WiFi.ready() && (! mqttClient->isConnected()) && (currentTime - lastAttempt > 30000)) {
        
        debug_message(String::format("Attempting MQTT broker connection, WiFi signal strength: %d dBm...", sigStrength), true);
        
        // Attempt to connect
        if (mqttClient->connect(mqttClientId)) {
            debug_message("Connected to MQTT broker", true);
            
            // Resubscribe to the control topic.
            mqttClient->subscribe(mqttControlInputTopic);
        }
        else {
            debug_message("MQTT broker connection failed, retry again in 30 seconds");
        }
        
        lastAttempt = currentTime;
    }

}

// The function to be called whenever a message comes in on subscribed topics.
void mqtt_callback(char* topic, byte* payload, unsigned int length) {

    // The payload may not be NULL terminated so keep that in mind
    // when working with it, always use the length.
    
    // Make a copy of the payload to a new JSON string.  It needs to be
    // writable because the JSON parser with add NULL terminators and
    // will replace escaped characters.
    char jsonString[length + 1];

    for (int i = 0; i < length; i++) {
        jsonString[i] = (char) payload[i];
    }
    
    jsonString[length] = NULL;
    
    debug_message(String::format("Message rcvd: [%s] %s", topic, jsonString));

    DynamicJsonBuffer jsonBuffer;
    
    JsonObject& root = jsonBuffer.parseObject(jsonString);

    // Check to make sure JSON string could be parsed
    if (! root.success()) {
        debug_message("Unable to parse JSON string");
        
        // Quickly flash the LED to indicate JSON parsing error
        set_led_color(JSON_ERR);
        delay(50);
        
        return;
    }

    // Check for an led key.  If present, turn on the LED if the value is a 1, and turn it off
    // if the value is 0.
    if (root.containsKey("led")) {
        if (root["led"] == 1) {
            // Turn the LED on.
            digitalWrite(BUILTIN_LED, HIGH);
        }
        else if (root["led"] == 0) {
            // Turn the LED off.
            digitalWrite(BUILTIN_LED, LOW); 
        }    
    }
    
    // Check for a reset key.  If present, reset the device if the value is a 1.
    if (root.containsKey("reset")) {
        if (root["reset"] == 1) {
            // Reset the device.
            System.reset();
        }
    }

}

// Wind speed ISR.  The wind speed sensor closes a magnetic reed switch twice per revolution.  There is no pullup on the
// SparkFun shield so the Photon internal input pullup must be enabled on this pin.  Therefore, the closure of the reed
// switch will generate a falling edge interrupt.  A wind speed of 1.492 mph will generate a pulse once per second.
void wind_speed_isr() {

    static unsigned long lastWindIRQ = 0;
    unsigned long currentTime = millis();

#ifdef DEBUG_WIND_SPEED
    static bool toggleTriggerOut = false;

    toggleTriggerOut = ! toggleTriggerOut;
    digitalWriteFast(WIND_SPEED_TRIG, toggleTriggerOut);

    windRaw++;
#endif

    // Ignore any switch bounce glitches within 10 msec after the initial reed switch closure.  This means the maximum
    // measureable speed is 149.2 mph, so this limit should not be a problem.
    if (currentTime - lastWindIRQ > 10) {
        
#ifdef DEBUG_WIND_SPEED
        windDebounced++;
#endif
        
        // Make sure the input is still LOW in the interrupt routine.  The measured interrupt latency is around 6 usec.  If
        // the input is HIGH again by the time we get to the interrupt routine 6 usec later, then the interrupt was probably
        // triggered by noise.  The measured duty cycle of the input was about 40% LOW and 60% HIGH when the sensor is
        // rotating.  This means the input will be LOW for a minimum of 4 msec at maximum measureable speed.
        if (pinReadFast(WIND_SPEED) == LOW) {
            
            // Save the current time for the next IRQ
            lastWindIRQ = currentTime;
            
            // Increment the wind tick counter.  The actual measurement will compare current tick with previous ticks over
            // the time period to calculate the average wind speed over that time period.
            windTicks++;
            
        }
        
    }

}

// Rainfall ISR.  The rain sensor closes a magnetic reed switch every time the bucket tips.  A tip of the bucket represents
// 0.011" of rainfall.  There is no pullup on the SparkFun shield so the internal input pullup must be enabled on this pin.
// Therefore, the closure of the reed switch will generate a falling edge interrupt.
void rain_isr() {

    static unsigned long lastRainIRQ = 0;
    unsigned long currentTime = millis();

#ifdef DEBUG_RAIN
    static bool toggleTriggerOut = false;

    toggleTriggerOut = ! toggleTriggerOut;
    digitalWriteFast(RAIN_TRIG, toggleTriggerOut);

    rainRaw++;
#endif

    // Ignore any switch bounce glitches within 10 msec after the initial reed switch closure.  Even at an extremely
    // high rainfall of 5" per hour, the bucket would only tip about every 8 seconds.
    if (currentTime - lastRainIRQ > 10) {
        
#ifdef DEBUG_RAIN
        rainDebounced++;
#endif
        
        // Make sure the input is still LOW in the interrupt routine.  The measured interrupt latency is around 6 usec.  If
        // the input is HIGH again by the time we get to the interrupt routine 6 usec later, then the interrupt was probably
        // triggered by noise.  The measured LOW time of the input during a bucket tip was measured to be between 80 and
        // 120 msec.
        if (pinReadFast(RAIN) == LOW) {
            
            // Save the current time for the next IRQ
            lastRainIRQ = currentTime;
            
            // Increment the accumulated rainfall.  This number represents thousaunds of an inch.  Each tip of the bucket
            // equall 0.011" of rainfall.
            rainAccum += 11;
            
        }
        
    }

}

// Get the weather readings.  This includes temp, humidity, pressure, and wind.  Rainfall is handled in a different routine.
void get_weather(State currentState) {
    
    // Variables for determining when sensors should be read again
    static unsigned long previousReadTime = 0;
    unsigned long currentTime = millis();

    // Check to see if it is time to read the sensors
    if (currentTime - previousReadTime >= READ_INTERVAL) {
        
        // Set the LED color to indicate an active sensor read.  The reads
        // take about 200 msec so we should see the LED.
        set_led_color(READ_SENSORS);
        
        // Measure Relative Humidity from the HTU21D
        float humidity = sensor.getRH();
        
        // Measure Temperature from the HTU21D in Fahrenheit
        float temp = sensor.getTempF();
        
        // Measure the Barometer temperature in Fahrenheit from the MPL3115A2
        float baroTemp = sensor.readBaroTempF();
        
        // Measure Pressure from the MPL3115A2 in Pascals
        float pascals = sensor.readPressure();
        
        // If in altitude mode, you can get a reading in feet with this line
        // float altitude = sensor.readAltitudeFt();
        
        // Get the wind direction
        float windDir = get_wind_direction();
        
        // Get the wind speed
        float windSpeed = get_wind_speed();
        
        // Save the last read time
        previousReadTime = currentTime;
        
        // Encode the message in JSON.
        
        // First create the JSON buffer and the root object.  Use the dynamic buffer instead of static.
        // The static version uses the stack and the dynamic version uses the heap.  I was having issues
        // with a static size of 200 not being big enough.  Therefore use the dynamic so the size is not
        // an issue.
        
        // StaticJsonBuffer<200> jsonBuffer;
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        
        // Put the data into the JSON object.
        root.add("temperature").set(temp, 3);
        root.add("humidity").set(humidity, 3);
        
        JsonObject& barometer = root.createNestedObject("barometer");
        barometer.add("temperature").set(baroTemp, 3);
        barometer.add("pressure").set(pascals, 2);
        // barometer.add("altitude").set(altitude, 2);
        
        JsonObject& wind = root.createNestedObject("wind");
        wind.add("direction").set(windDir, 1);
        wind.add("speed").set(windSpeed, 3);
        
#ifdef DEBUG_WIND_SPEED
        JsonObject& debug = wind.createNestedObject("debug");
        debug["raw"] = windRaw;
        debug["debounced"] = windDebounced;
        debug["ticks"] = windTicks;
#endif
        
        root.add("wifi_sig").set(WiFi.RSSI(),0);
        
        // Turn the JSON structure into a string for sending over the MQTT topic.
        char jsonOutputBuffer[255];
        root.printTo(jsonOutputBuffer, sizeof(jsonOutputBuffer));
        
        if ((currentState == NO_CLOUD || currentState == GOOD) && WiFi.ready() && mqttClient->isConnected()) {
            
            // Publish the message on the MQTT topic, but only if we are connected to the broker.
            if (mqttClient->publish(mqttSensorOutputTopic, jsonOutputBuffer))
                debug_message(String::format("Message sent: [%s] %s", mqttSensorOutputTopic.c_str(), jsonOutputBuffer));
            
        }
        
    }

}

// Get the wind speed.  This routine relies on the the wind speed IRQ to continuously increment the wind speed ticks.
// This routine is meant to be called on a periodic interval.  It keeps track of the time and number of ticks betwween
// calls to calculate the average wind speed over that interval.
float get_wind_speed() {
    
    // Keep track of the last wind speed measurement time and ticks
    static unsigned long lastWindSpeedCheck = 0;
    static unsigned int lastWindTicks = 0;
    
    // Snapshot the current time and current wind tick count
    unsigned long currentTime = millis();
    unsigned int currentWindTicks= windTicks;

    float windSpeed;
    
    // Calculate the avg wind speed in mph over the interval.
    // windSpeed = (1.492 mph/sec * numTicks) / (time_in_msec / 1000.0 msec/sec)
    windSpeed = 1492.0 * (currentWindTicks - lastWindTicks) / (currentTime - lastWindSpeedCheck);
    
    // Update the last measurement time and ticks for the next time this routine is called
    lastWindSpeedCheck = currentTime;
    lastWindTicks = currentWindTicks;
    
    return windSpeed;

}

// Read the wind direction sensor.  Returns the wind heading in degrees
float get_wind_direction() {
    
    unsigned int adc;
    float direction;

    // Read the analong voltage from the sensor resistor divider
    adc = analogRead(WIND_DIR);

#ifdef DEBUG_WIND_DIR
    debug_message(String::format("Wind direction raw ADC: %d", adc));
#endif    

    // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
    // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
    // Note that these are not in compass degree order.  See Weather Meters datasheet for more information.

    if (adc < 1455)       return (-1);     // Sensor shorted or no power
    else if (adc <= 1524) direction = 112.5;  // ESE
    else if (adc <= 1577) direction = 67.5;   // ENE
    else if (adc <= 1658) direction = 90.0;   // E
    else if (adc <= 1827) direction = 157.5;  // SSE
    else if (adc <= 2035) direction = 135.0;  // SE
    else if (adc <= 2209) direction = 202.5;  // SSW
    else if (adc <= 2461) direction = 180.0;  // S
    else if (adc <= 2723) direction = 22.5;   // NNE
    else if (adc <= 2986) direction = 45.0;   // NE
    else if (adc <= 3208) direction = 247.5;  // WSW
    else if (adc <= 3334) direction = 225.0;  // SW
    else if (adc <= 3515) direction = 337.5;  // NNW
    else if (adc <= 3655) direction = 0.0;    // N
    else if (adc <= 3763) direction = 292.5;  // WNW
    else if (adc <= 3884) direction = 315.0;  // NW
    else if (adc <= 4002) direction = 270.0;  // W
    else                  return (-2);     // Sensor unplugged or open

    return ((float) (((int) ((direction + WIND_DIR_OFFSET) * 10.0)) % 3600)) * 0.1;

}

// Update the rainfall values and historical arrays
void get_rainfall(State currentState) {
    
    // Keep track of the minute the last update occured.  Initialize to an invalid
    // minute so the update is ran the first time this function is called.
    static int lastUpdateMinute = -1;

    // Update the rain totals every minute
    int currentTime = Time.now();
    int currentMinute = Time.minute(currentTime);
    
    if (currentMinute != lastUpdateMinute) {
        
        // The minute changed, get the accumulated rainfall value in thousandths of an inch.
        unsigned int currentAccumRain = rainAccum;
        
        // Set the LED color to indicate a rainfall update.  Delay so we can see the
        // LED, otherwise this routine completes in a little over 1 msec and we would
        // not see the LED flash before changing back.
        
        set_led_color(UPDATE_RAIN);
        delay(250);
        
        // Get the current hour and day of the week
        int currentHour = Time.hour(currentTime);
        int currentDay = Time.weekday(currentTime) - 1;
        
        // Calculate the rainfall in the last hour and update the array for the last minute
        rainLastHr = 0.001 * (currentAccumRain - rainAccumMinute[currentMinute]);
        rainAccumMinute[currentMinute] = currentAccumRain;
        
        // Calculate the rainfail in the last 24 hours and update the array for the current hour
        rainLast24Hr = 0.001 * (currentAccumRain - rainAccumHour[(currentHour + 1) % 24]);
        rainAccumHour[currentHour] = currentAccumRain;
        
        // Update the end of day rain accumulation for the previous day, when the day changes
        if (currentHour == 0 && currentMinute == 0) {
            
            // The Photon version of the modulo operation will return negative numbers.  Make
            // sure the number is positive by adding the modulus before doing the modulo operation.
            rainAccumDay[(currentDay - 1 + 7) % 7] = currentAccumRain;
            
        }
        
        // Update the 7-day daily rain totals
        for (int i = 0; i < 7; i++) { 
            
            int accumCalc;
            
            if (i == 0)
                accumCalc = currentAccumRain;
            else
                // The Photon version of the modulo operation will return negative numbers.  Make
                // sure the number is positive by adding the modulus before doing the modulo operation.
                accumCalc = rainAccumDay[(currentDay - i + 7) % 7];
            
            // The Photon version of the modulo operation will return negative numbers.  Make
            // sure the number is positive by adding the modulus before doing the modulo operation.
            rainDaily[i] = 0.001 * (accumCalc - rainAccumDay[(currentDay - i - 1 + 7) % 7]);
            
        }
        
        // Update the last minute read for the next time this funciton is called
        lastUpdateMinute = currentMinute;
        
        // Encode the message in JSON.
        
        // First create the JSON buffer and the root object.  Use the dynamic buffer instead of static.
        // The static version uses the stack and the dynamic version uses the heap.  I was having issues
        // with a static size of 200 not being big enough.  Therefore use the dynamic so the size is not
        // an issue.
        
        // StaticJsonBuffer<200> jsonBuffer;
        DynamicJsonBuffer jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        
        // Put the data into the JSON object.
        JsonObject& rain = root.createNestedObject("rain");
        rain.add("lastHr").set(rainLastHr, 3);
        rain.add("last24Hr").set(rainLast24Hr, 3);
        
        JsonArray& daily = rain.createNestedArray("daily");
        for (int i = 0; i < 7; i++)
            daily.add(rainDaily[i], 3);
        
#ifdef DEBUG_RAIN
        JsonObject& debug = root.createNestedObject("debug");
        debug["raw"] = rainRaw;
        debug["debounced"] = rainDebounced;
        debug["ticks"] = rainAccum / 11;
#endif
        
        // Turn the JSON structure into a string for sending over the MQTT topic.
        char jsonOutputBuffer[255];
        root.printTo(jsonOutputBuffer, sizeof(jsonOutputBuffer));
        
        if ((currentState == NO_CLOUD || currentState == GOOD) && WiFi.ready() && mqttClient->isConnected()) {
            
            // Publish the message on the MQTT topic, but only if we are connected to the broker.
            if (mqttClient->publish(mqttSensorOutputTopic, jsonOutputBuffer))
                debug_message(String::format("Message sent: [%s] %s", mqttSensorOutputTopic.c_str(), jsonOutputBuffer));
            
        }
        
    }

}

// Set the LED color
void set_led_color(State color) {

    switch (color) {
        case SETUP:
            RGB.color(0, 0, 255);       // Blue
            break;
        case INIT:
            RGB.color(0, 0, 0);         // Off
            break;
        case NO_WIFI:
            RGB.color(255, 0, 0);       // Red
            break;
        case WEAK_SIG:
            RGB.color(255, 96, 0);     // Orange
            break;
        case NO_MQTT:
            RGB.color(255, 255, 0);     // Yellow
            break;
        case NO_CLOUD:
            RGB.color(255, 255, 255);   // White
            break;
        case GOOD:
            RGB.color(0, 255, 0);       // Green
            break;
        case READ_SENSORS:
            RGB.color(0, 128, 255);     // Medium Blue
            break;
        case UPDATE_RAIN:
            RGB.color(255, 0, 255);     // Magenta
            break;
        case EXIT_APP:
            RGB.color(0, 0, 0);         // Off
            break;
        case JSON_ERR:
            RGB.color(255, 96, 0);     // Orange
            break;
    }

}

// Update the time.  Make sure the timezone is set before calling this function the first time.
// This will resync the time to the cloud once per day.  It will also monitor when it is time
// to begin and end Daylight Savings Time.  It should be called in setup and and periodically
// in the loop
void update_time() {
    
    // Keep track of what the day of the week was when we last did a time sync to the cloud.  By
    // comparing the current day of the week with the last time it was done, we will ensure that
    // we only resync once per day.  Initialize to an invalid day of the week so it will
    // resync the first day.
    static int lastDayTimeSync = -1;
    
    // Get the current time in standard time regardless of whether DST is turned on or not.
    // Doing checks in standard time makes things easier than compensating for the overlapping
    // times during the change in the fall.  The Time.now() function always returns UTC time and
    // does not use the timezone or DST settings.  However, when the individual Time.day(),
    // Time.hour(), etc functions are called they do use the current timezone and DST settings
    // when reporting back their values.  This is true whether the individual functions are called
    // without an argument which uses the default Time.now() or is passed a specific time value.
    // This is why the individual functions report back times twice the timezone offset when passed
    // a Time.local() time setting.  So, if we fake out the Time.now() value by backing out the
    // DST seting before passing it to the individual functions, we will only get back standard
    // times after the individual functions add back in the DST setting.
    
    // Subtract an hour if DST is set so we only get standard times
    int currentTime = Time.now() - 3600 * Time.isDST();
    
    int month = Time.month(currentTime);
    int day = Time.day(currentTime);
    
    // Weekday should be between 0-7, 0 for Sunday, 6 for Saturday.  The Time.weekday() function
    // returns values between 1-7
    int weekday = Time.weekday(currentTime) - 1;
    
    int hour = Time.hour(currentTime);
    int minute = Time.minute(currentTime);
    int second = Time.second(currentTime);
    
    // Gets the day of the month of the previous Sunday or if today is Sunday it gets the current
    // day of the month
    int previousSunday = day - weekday;
    
    bool inDST;
    
    // Resync the time 30 seconds after midnight every day.  We should not be doing any time of day
    // dependent operations during this time.  The sync should happen at boot also.
    if (lastDayTimeSync != weekday && hour == 0 && minute == 0 && second >= 30) {
        
        debug_message(String::format("Resyncing time to cloud: WiFi: %d, MQTT: %d, Cloud: %d...", WiFi.ready(), mqttClient->isConnected(), Particle.connected()));
        Particle.syncTime();
        debug_message("Time resync completed");
        lastDayTimeSync = weekday;
        
    }
    
    // Check whether we are in Daylight Savings Time
    
    // Before March or after November, we are not in DST
    if (month < 3 || month > 11)
        inDST =  false;
    
    // April to October, we are in DST
    else if (month > 3 && month < 11)
        inDST = true;
    
    // For March we need to do some extra checking
    else if (month == 3) {
        
        // The time change happens on the 2nd Sunday in March.  The second Sunday always occurs
        // on or between the 8th and 14th.  If the previous or current Sunday occurs on or after
        // the 8th then we are in DST.  However if today is the Sunday of the time change we need
        // to do some more checking.
        if (previousSunday >= 8) {
            
            // Today is the Sunday of the time change, we need to check the time of day
            if ((weekday == 0) && (day >= 8 && day <= 14)) {
                
                // Make the time change happen at 2:00:15 am.  Make it a few seconds after the hour
                // so the rainfall updates can complete before the time changes.  The time change
                // will occur at 2:00 am standard time (3:00 am DST)
                
                // At or past 3:00:00 am, so we are in DST
                if (hour > 2)
                    inDST = true;
                
                // Before 2:00:00 am, so we are not in DST
                else if (hour < 2)
                    inDST = false;
                
                // In the 2 am hour so we need to check the minutes and seconds
                else {
                    
                    // In the 2:00 am minute so we need to check the seconds
                    if (minute == 0) {
                        
                        // At or past 2;00:15 am so we are in DST
                        if (second >= 15)
                            inDST = true;
                        
                        // Before 2:00:15 am so we are not quite yet in DST
                        else
                            inDST = false;
                        
                    }
                    
                    // At or past 2:01:00 am so we are in DST
                    else
                        inDST = true;
                    
                }
            }
            
            // The previous Sunday occured on or after the 8th and today is not the Sunday of the time
            // change, so we are in DST
            else
                inDST = true;
            
        }
        
        // The previous or current Sunday was before the 8th so we are not in DST
        else
            inDST = false;
        
    }
    
    // For November we need to do some extra checking
    else {
        
        // The time change happens on the 1st Sunday in November.  The first Sunday always occurs
        // on or between the 1st and 7th.  If the previous or current Sunday occurs before the 1st,
        // then we are in DST.  However if today is the Sunday of the time change we need
        // to do some more checking.
        
        // The previous or current Sunday occurred before the 1st, so we are in DST
        if (previousSunday < 1)
            inDST = true;
        
        // Today is the Sunday of the time change, we need to check the time of day
        else if ((weekday == 0) && (day >= 1 && day <= 7)) {
            
            // Make the time change happen at 1:59:45 am DST.  Make it a few seconds before the hour
            // so the rainfall updates can complete after the time changes.  The time change
            // will occur at 1:00 am standard time which is what the time calculations are based
            // upon.
            
            // At or past 1:00:00 am (2:00:00 am DST), so we are not in DST
            if (hour > 0)
                inDST = false;
            
            // In the 0 am (1 am DST) hour so we need to check the minutes and seconds
            else {
                
                // In the 0:59 am (1:59 am DST) minute so we need to check the seconds
                if (minute == 59) {
                    
                    // At or past 0;59:45 am (1:59:45 am DST) so we are not in DST
                    if (second >= 45)
                        inDST = false;
                    
                    // Before 0:59:45 am (1:59:45 am DST) so we are still in DST for a few seconds longer
                    else
                        inDST = true;
                    
                }
                
                // Before 0:59:00 am (1;59:00 am DST) so we are in DST
                else
                    inDST = true;
                
            }
            
        }
        
        // The previous or current Sunday occured on or after the 1st and today is not the Sunday of the time
        // change, so we are not in DST
        else
            inDST = false;
        
    }
    
    // If the DST state has changed then update it
    if ((bool) inDST != (bool) Time.isDST()) {
        if (inDST) {
            debug_message("Setting Daylight Savings Time", true);
            Time.beginDST();
        }
        else {
            debug_message("Setting Standard Time", true);
            Time.endDST();
        }
    }
    
}

// Send debug messages to serial port and/or Particle cloud
inline void debug_message(String msg, bool cloud) {

#ifdef DEBUG_SERIAL

    Serial.println(msg);

#endif

#ifdef DEBUG_CLOUD

    if (cloud) {
        Particle.publish("app/debug", msg);
        Particle.process();
    }
    
#endif

}
