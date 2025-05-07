
#include <WiFiClient.h>
#include <TelnetStream.h>
#include "driver/adc.h"
#include <esp_wifi.h>
//#include <esp_bt.h>

#define HAS_ENCODER  //uncomment this line if you are using the rotary encoder for menu display
//rotary encoder
#ifdef HAS_ENCODER
#include "FastInterruptEncoder.h"  //https://github.com/levkovigor/FastInterruptEncoder
#define ROTARY_ENCODER_A_PIN 15
#define ROTARY_ENCODER_B_PIN 4
#define ROTARY_ENCODER_BUTTON_PIN 16

//Encoder enc(PA0, PA1, SINGLE /* or HALFQUAD or FULLQUAD */, 250 /* Noise and Debounce Filter (default 0) */); // - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
Encoder enc(ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_A_PIN, HALFQUAD, 0);  // - Example for ESP32

unsigned long encodertimer = 0;
bool buttonClicked = false;

/* create a hardware timer */
hw_timer_t* timer = NULL;

void IRAM_ATTR Update_IT_callback()  //callback for encoder interrupt
{
  enc.loop();
}
void IRAM_ATTR buttonISR() {  //callback for encoder button when click released
  static unsigned long lastTimePressed = 0;
  if ((millis() - lastTimePressed) > 100)  //debounce
  {
    buttonClicked = true;
    //Serial.println("button clicked"); //do not uncomment... to keep real time during interrupt
    lastTimePressed = millis();
  }
}

bool isEncoderButtonClicked() {  // eats the clcik release event
  if (buttonClicked) {
    buttonClicked = false;
    return true;
  } else return false;
}
#endif


int nbTimeout = 0;
int timeOut = 0;
int nbAlarm = 0;


//Json
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson


//OLED
#define OLED  // you may comment if OLED not used

String res;

#define OLED_SDA_PIN 17  // i2c pins for Oled
#define OLED_SCL_PIN 5

String displayDebug;
long lastOled;

int OLEDDisplayTimeout = 5;  // oled menu display timeout (seconds)

#define OLED_ADDR 0x3C  // OLED i2c address

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// oled menu
const byte menuMax = 5;       // max number of menu items
const byte lineSpace1 = 9;    // line spacing (6 lines)
const byte lineSpace2 = 16;   // line spacing (4 lines)
String menuOption[menuMax];   // options displayed in menu
byte menuCount = 0;           // which menu item is curently highlighted
String menuTitle = "";        // current menu ID number (blank = none)
byte menuItemClicked = 100;   // menu item has been clicked flag (100=none)
uint32_t lastREActivity = 0;  // time last activity was seen on rotary encoder
bool firstLaunch = true;
bool reButtonState = HIGH;
int encoder0Pos;
int itemTrigger = 1;
bool clicked = false;

// oled SSD1306 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


//sensors selection
#define HAS_FRONT_SCALE
#define HAS_BACK_SCALE
//#define HAS_WIFI        //leave this line commented if you only want Bluetooth Low Energy ; uncomment for both Wifi and BLE

//wifi option
//click on the encoder button while booting the ESP32 to launch a Wifi access point
// you can change name and password here
const char* APssid = "JP RC_CoG_finder";
const char* APpassword = "";

#define PIN_CONF 13  //CONF pin on the board. Used to load wifi manager during boot (to enter credentials for your ESP32 to connect to your wifi router)


//front scale
#define PIN_CLOCK 14  //output to generate clock on Hx711
#define PIN_DOUT 12   //input Dout from Hx711

//back scale
#define PIN_CLOCK2 32      //output to generate clock on Hx711_2
#define PIN_DOUT2 33       //input Dout from Hx711_2
bool doCalibZero = false;  //calibration should not be done into Bluetooth call back...
long calibZero = 0;        //No load front scale Output
long calib = 130968;       //sensor output - calibZero for Weight calibration --> will be auto calibrated later
int calibWeight = 1000;    //weight at which calibration is done --> expressed in grams.
float AverageWeight = 0;
float CurrentRawWeight = 0;

long calibZero2 = 0;      //No load back scale sensor Output
long calib2 = 130968;     //sensor output - calibZero2 for Weight calibration --> will be auto calibrated later
int calibWeight2 = 1000;  //weight at which calibration is done --> expressed in grams.
float AverageWeight2 = 0;
float CurrentRawWeight2 = 0;

int length;            //scale lever length
float CoG, targetCoG;  //measured Center of Gravity and targetCoG value
int L1;                //nose to LE distance where to add weight
int L2;                //tail to LE distance

#define LED_PIN 22

//Preferences
#include <Preferences.h>
Preferences preferences;

//touchpad
touch_pad_t touchPin;
int threshold = 40;  //Threshold value for touchpads pins

boolean TouchWake = false;  // if boot with touchpad

//WifiManager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager


bool shouldSaveConfig = false;  //flag for saving data
bool touch3detected = false;    //touch3 used to launch WifiManager (hold it while reseting)

//callback notifying us of the need to save config after wifi manager
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


String ssid = "";
String password = "";
boolean hasWifiCredentials = false;


//#define W_DEBUG     //debug Wifi
#define DEBUG_OUT
//#define xDEBUG
#define E_DEBUG    //debug rotary encoder
#define UDP_DEBUG  //debug UDP messages
//#define TEST
#define PREFERENCES_DEBUG
//#define DEBUG_TELNET      //debug using telnet
#define DEBUG_BLE
//#define RAW_WEIGHT_DEBUG  //debug scales
#define DEBUG_CAL0
//#define DEBUG_W


//UDP --------------            used for communication with Android App
unsigned int localPort = 5000;  // local port to listen on
char packetBuffer[64];          //buffer to hold incoming packet
char AndroidConnected = 0;
long Timeout;
String device = "RC_CoG_finder";
String theMAC = "";

WiFiUDP Udp;
long LastUDPnotification;
//end UDP-----------

//**************
//BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

long LastBLEnotification;

//BLE declarations
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c3319160"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26bb"

void BLEnotify(String theString) {
  // if (deviceConnected == true)
  {
#ifdef xDEBUG_BLE
    Serial.print("BLE notify : ");
    Serial.println(theString);
#endif
    char message[21];
    String small = "";  //BLE notification MTU is limited to 20 bytes
    while (theString.length() > 0) {
      small = theString.substring(0, 19);  //cut into 20 chars slices
      theString = theString.substring(19);
      small.toCharArray(message, 20);
      pCharacteristic->setValue(message);
      pCharacteristic->notify();
      delay(3);                        // bluetooth stack will go into congestion, if too many packets are sent
      LastBLEnotification = millis();  //will prevent to send new notification before this one is not totally sent
    }
  }
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
#ifdef DEBUG_BLE
    Serial.println("client connected");
#endif
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
#ifdef DEBUG_BLE
    Serial.println("client disconnected");
#endif
    // Start advertising
    pServer->getAdvertising()->stop();
    delay(100);
    pServer->getAdvertising()->start();
#ifdef DEBUG_BLE
    Serial.println("Waiting a client connection to notify...");
#endif
    delay(100);
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    String test = "";
    if (rxValue.length() > 0) {
#ifdef DEBUG_OUT
      Serial.print("Received : ");
#endif
      for (int i = 0; i < rxValue.length(); i++) {
#ifdef DEBUG_OUT
        Serial.print(rxValue[i]);
#endif
        test = test + rxValue[i];
      }
#ifdef DEBUG_OUT
      Serial.println();
#endif
    }
    readCmd(test);
  }
};


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  pinMode(PIN_CONF, INPUT_PULLUP);


#ifdef HAS_ENCODER
  pinMode(ROTARY_ENCODER_BUTTON_PIN, INPUT_PULLUP);               //rotary encoder push button must be pulled up
  attachInterrupt(ROTARY_ENCODER_BUTTON_PIN, buttonISR, RISING);  //will detect the click release event and launch buttonISR function
#endif

  // initialise the oled display
#ifdef OLED
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);  // if you get an error it may be the board you are using does not allow defining the pins in which case try:  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(("\nError initialising the oled display"));
  }

  // Display splash screen on OLED
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, lineSpace2 * 2);
  display.setTextSize(2);
  display.print("CoGfinder");
  display.display();
  delay(2500);
#endif

  //BLE
  // Create the BLE Device
  BLEDevice::init("JP CoGfinder");

  // Create the BLE Server
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
#ifdef DEBUG_BLE
  Serial.println("Waiting a client connection...");
#endif


  delay(100);

  Serial.println("***************");
  Serial.println("program started");
  Serial.println("***************");
  Serial.println(" ");


  //Preferences
  preferences.begin("RC_CoG_finder", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only

  calibWeight = preferences.getInt("calibWeight", 500);  //by default calibration of load cell is done with 500g load attached to the front scale
  calib = preferences.getLong("calib", -222444);         //will be computed during calibration menu

  calibWeight2 = preferences.getInt("calibWeight2", 500);  //by default calibration of load cell is done with 500g load attached to the back scale
  calib2 = preferences.getLong("calib2", -222444);         //will be computed during calibration menu

  length = preferences.getInt("length", 200);  //by default calibration of lever length is 200mm can be changed using menu
  L1 = preferences.getInt("L1", 100);          //by default calibration of L1 is 100mm
  L2 = preferences.getInt("L2", 800);          //by default calibration of L2 is 800mm
  targetCoG = preferences.getInt("CoG", 80);   //by default to 80mm

  ssid = preferences.getString("ssid", "");  // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "");

#ifdef PREFERENCES_DEBUG
  Serial.println("_________________");
  Serial.println("read preferences :");
  Serial.print("calib front scale : ");
  Serial.println(calib);
  Serial.print("calib weight (g) : ");
  Serial.println(calibWeight);
  Serial.print("calib back scale : ");
  Serial.println(calib2);
  Serial.print("calib2 weight (g) : ");
  Serial.println(calibWeight2);
  Serial.println("_________________");
#endif
  //preferences.end();  // Close the Preferences

#ifdef HAS_WIFI

  // click on encoder button during boot to launch an Access point for the Android to connect
  if (digitalRead(ROTARY_ENCODER_BUTTON_PIN) == LOW) {
    // setup Wi-Fi network with SSID and password
    Serial.printf("Setting AP (Access Point)… '%s'\n", APssid);
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(APssid, APpassword);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.print("UDP started ");
#ifdef OLED
    display.clearDisplay();
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.print("AP : JP RC_CoGfinder");
    display.display();
    delay(4000);
#endif
  } else  // regular use of a Wifi network on which ESP32 is connected
  {
#ifdef OLED
    display.clearDisplay();
    display.setCursor(0, 20);
    display.setTextSize(1);
    display.print("Connect to Wifi");
    display.display();
#endif


    if (digitalRead(PIN_CONF) == LOW)  //short CONF pins to launch wifimanager
    {
#ifdef OLED
      display.clearDisplay();
      display.setCursor(0, 20);
      display.setTextSize(1);
      display.print("WifiManager");
      display.setCursor(0, 40);
      display.print("enter credentials");
      display.display();
      delay(2000);
#endif


      //  //connect to WiFi
      //WiFiManager
      //Local intialization. Once its business is done, there is no need to keep it around
      WiFiManager wifiManager;

      //set config save notify callback
      wifiManager.setSaveConfigCallback(saveConfigCallback);
      //reset settings - for testing
      //wifiManager.resetSettings();
      //sets timeout until configuration portal gets turned off
      //useful to make it all retry or go to sleep
      //in seconds
      wifiManager.setTimeout(300);


      //fetches ssid and pass and tries to connect
      //if it does not connect it starts an access point with the specified name
      //here  "JP RCCoGfinder"
      //and goes into a blocking loop awaiting configuration
      if (!wifiManager.startConfigPortal("JP RC_CoG_finder")) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(5000);
      }

      delay(2000);
      //  //save the custom WifiManager's parameters if needed
      if (shouldSaveConfig) {
        Serial.println("saving Wifi credentials ");
        //read updated parameters
        preferences.putString("password", WiFi.psk());
        preferences.putString("ssid", WiFi.SSID());
        delay(2000);
        ESP.restart();
        delay(5000);
      }
    }

    //connect to WiFi
    WiFi.begin(ssid.c_str(), password.c_str());
    long start = millis();
    hasWifiCredentials = false;

    while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 20000)) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;
  }

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);

  //if (hasWifiCredentials) //don't block the prog if no wifi...it could work with only OLED display
  {
    TelnetStream.begin();  //used to debug over telnet
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    //Start UDP
    Udp.begin(localPort);
  }

#endif  //end HAS_WIFI

  Serial.println(" ");
  Serial.println("start monitoring sensors : \n");
#ifdef OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 30);
  display.print("scales tare");
  display.display();
#endif

  getCalibZero();  // tare the two scales

#ifdef DEBUG_TELNET
  TelnetStream.print("start");
#endif

#ifdef HAS_ENCODER
  //we must initialize rotary encoder (don't do it before wifi is launched... or crash !)
  if (enc.init()) {
    Serial.println("Encoder Initialization OK");
  } else {
    Serial.println("Encoder Initialization Failed");
    while (1)
      ;
  }

  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &Update_IT_callback, true);
  /* Set alarm to call onTimer function every 100 ms -> 100 Hz */
  timerAlarmWrite(timer, 10000, true);
  /* Start an alarm */
  timerAlarmEnable(timer);
#endif


  LastUDPnotification = millis();
}

//***********************************************************************************************************************
void loop() {
// esp_task_wdt_reset();  //reset the watchdog

// if a oled menu is active service it
#ifdef HAS_ENCODER
  menuCheck();             // check if encoder selection button is pressed
  if (menuTitle != "") {   // if a menu is active
    menuItemSelection();   // check for change in menu item highlighted
    staticMenu();          // display the menu
    menuItemActions();     // act if a menu item has been clicked
  } else displayValues();  // if no menu active,then will display realtime acquisitions
#endif
    //*******************
    //acquire all sensors
    //*******************

    //front scale HX711_1
#ifdef HAS_FRONT_SCALE
  GetRawWeight();  //HX711 sensor
  AverageWeight = (calibZero - CurrentRawWeight) * calibWeight / (calib);
#ifdef DEBUG
  Serial.print("front scale weight = ");
  Serial.print(AverageWeight);
  Serial.println(" g");
#endif
#endif  //HAS_FRONT_SCALE

  //back scale HX711_2
#ifdef HAS_BACK_SCALE
  GetRawWeight2();  //HX711 sensor
  AverageWeight2 = (calibZero2 - CurrentRawWeight2) * calibWeight2 / (calib2);
#ifdef DEBUG
  Serial.print("back scale weight2 = ");
  Serial.print(AverageWeight2);
  Serial.println(" g");
#endif
#endif  //HAS_BACK_SCALE

  if ((AverageWeight + AverageWeight2) != 0) CoG = AverageWeight2 * length / (AverageWeight + AverageWeight2);
  else CoG = .01;  //avoid divide by zero

  if (doCalibZero)  //a way to perform tare outside the BLE callback
  {
    doCalibZero = false;
    getCalibZero();
  }

#ifdef HAS_WIFI
  //************
  // UDP process
  //************
  int packetSize = Udp.parsePacket();  //if there's data available, read a packet coming from Android phone
  if (packetSize) {
    Timeout = millis();  //rearm software watchdog

#if defined xxxDEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
#endif
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
#if defined UDP_DEBUG
    Serial.print("UDP Contents: ");
    Serial.println(packetBuffer);
#endif
    String Res;
    String test;
    int i;
    test = packetBuffer;

    if (test == "ID")  // send a reply, to the IP address and port that sent us the packet we received
    {
      AndroidConnected = 1;
      timeOut = millis();
      Res = device;
      sendUDP(Res);

    } else if (test.startsWith("TOPIC"))  // send a reply, to the IP address and port that sent us the packet we received
    {
      //{"sender":"18FE349E7690","device":"WaterLeak"}
      Res = WiFi.macAddress();
      Res.replace(":", "");
      sendUDP(Res);
    }

    else if (test.startsWith("{"))  // decode json
    {
      // message = test.substring(6);
      readCmd(test);
    }

    else if (test.startsWith("STOP"))  // send a reply, to the IP address and port that sent us the packet we received
    {
      sendUDP("okSTOP");
    }
  }

#endif  //end HAS_WIFI


  //send sensors data as fast as possible (else uncomment the following line)
  if (((millis() - LastUDPnotification) > 300))  // send UDP message to Android App (no need to be connected, a simple notification, send and forget)
  {
    res = "{\"F\": " + String(AverageWeight) + ",\"B\": " + String(AverageWeight2) + ",\"L\": " + String(length) + ",\"L1\": " + String(L1) + ",\"L2\": " + String(L2) + ",\"C\": " + String(CoG) + ",\"TC\": " + String(targetCoG) + "}";
    LastUDPnotification = millis();
    BLEnotify(res);  //try to send via Bluetooth
#ifdef DEBUG_TELNET
    TelnetStream.println(res);
#endif
#ifdef HAS_WIFI
    if (deviceConnected == false) sendUDP(res);  //try to send via UDP if not connected to BLE
#endif
  }





  if (((millis() - timeOut) > 6000) && (AndroidConnected == 1)) {
    AndroidConnected = 0;
    Serial.println("lost connection with Android phone, switch to Manual mode");
  }
}  //end of Loop



#ifdef OLED
void displayValues(void) {
  // Display values on OLED
  if ((millis() - lastOled) > 80)  // to keep display readible
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setCursor(0, lineSpace1 * 0);
    display.setTextSize(1);
    display.print("CoG ");
    display.print(CoG);
    display.print("mm");
    display.setCursor(0, lineSpace1 * 1);
    display.print("targetCoG ");
    display.print(targetCoG);
    display.print("mm");

    display.setCursor(0, lineSpace1 * 3);
    display.print("Weight ");
    display.print(AverageWeight + AverageWeight2);
    display.print("g");
    display.setCursor(0, lineSpace1 * 4);
    display.print("Wf ");
    display.print(AverageWeight);
    display.print(" Wb ");
    display.print(AverageWeight2);
    display.print("g");
    display.setCursor(0, lineSpace1 * 6);
    // compute weight to add to tail or nose
    float addWeight;
    if ((CoG - targetCoG) > 0) {
      addWeight = (length * AverageWeight2 - (targetCoG * (AverageWeight + AverageWeight2))) / (L1 + targetCoG);
      display.print("add ");
      display.print(addWeight);
      display.print("g to nose");
    } else {
      addWeight = (length * AverageWeight2 - (targetCoG * (AverageWeight + AverageWeight2))) / (-L2 + targetCoG);
      display.print("add ");
      display.print(addWeight);
      display.print("g to tail");
    }

    display.display();
    lastOled = millis();
  }
}
#endif

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int ftouchRead(int gpio)  // this will filter false readings of touchRead() function...
{
  int val = 0;
  int readVal;
  for (int i = 0; i < 10; i++) {
    readVal = touchRead(gpio);
    val = max(val, readVal);
  }
  return val;
}


void sendUDP(String Res) {
  char ReplyBuffer[Res.length() + 1];  // a string to send back
  Res.toCharArray(ReplyBuffer, Res.length() + 1);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(ReplyBuffer);  //was write...
  Udp.endPacket();
}

void readCmd(String test) {
  if (test.startsWith("{"))  //then it may contain JSON
  {
    StaticJsonDocument<2000> doc;
    DeserializationError error = deserializeJson(doc, test);
    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      Serial.println("deserializeJson() failed");  //answer with error : {"answer" : "error","detail":"decoding failed"}
      sendUDP("{\"answer\" : \"error\",\"detail\":\"decoding failed\"}");
    } else {
      // Fetch values --> {"Cmd":"CalF"}
      String Cmd = doc["Cmd"];
      if (Cmd == "Cal0")  //lever Length calibration
      {
        Serial.println("tare scales... ");
        doCalibZero = true;      //getCalibZero(); will be done into the main loop
      } else if (Cmd == "CalF")  //front scale calibration
      {
#ifdef HAS_FRONT_SCALE
        String value = doc["value"];
        calibWeight = value.toInt();
        //GetRawWeight();  //HX711 sensor is already acquired in the main loop
        calib = calibZero - CurrentRawWeight;
        if (calib == 0) calib = 130968;
        Serial.print("calibration front scale... ");
        Serial.println(calib);
        Serial.print(" for weight (g) ");
        Serial.println(calibWeight);

        preferences.putLong("calib", calib);
        preferences.putInt("calibWeight", calibWeight);
#endif
      } else if (Cmd == "CalB")  //back scale calibration
      {
#ifdef HAS_BACK_SCALE
        String value = doc["value"];
        calibWeight2 = value.toInt();
        //GetRawWeight2();  //HX711_2 sensor already acquired into the main loop
        calib2 = calibZero2 - CurrentRawWeight2;
        if (calib2 == 0) calib2 = 130968;
        Serial.print("calibration back scale... ");
        Serial.println(calib2);
        Serial.print(" for weight (g) ");
        Serial.println(calibWeight2);

        preferences.putLong("calib2", calib2);
        preferences.putInt("calibWeight2", calibWeight2);
#endif
      } else if (Cmd == "SetL")  //lever Length calibration
      {

        String value = doc["value"];
        length = value.toInt();
        Serial.print("set lever Length... ");
        Serial.print(length);
        Serial.println(" mm ");

        preferences.putInt("length", length);
      } else if (Cmd == "SetL1")  //lever Length calibration
      {

        String value = doc["value"];
        L1 = value.toInt();
        Serial.print("set L1 Length... ");
        Serial.print(L1);
        Serial.println(" mm ");

        preferences.putInt("L1", L1);
      } else if (Cmd == "SetL2")  //lever Length calibration
      {

        String value = doc["value"];
        L2 = value.toInt();
        Serial.print("set L2... ");
        Serial.print(L2);
        Serial.println(" mm ");

        preferences.putInt("L2", L2);
      } else if (Cmd == "SetCoG")  //lever Length calibration
      {

        String value = doc["value"];
        targetCoG = value.toInt();
        Serial.print("set target CoG... ");
        Serial.print(targetCoG);
        Serial.println(" mm ");

        preferences.putInt("CoG", targetCoG);
      }
    }
  }
}


#ifdef HAS_FRONT_SCALE
void GetRawWeight(void) {
  unsigned long RawWeight;
  // wait for the chip to become ready
  long startTime;
  //delay(5000);             //let the HX711 warm up
  AverageWeight = 0;

  startTime = millis();

  while ((digitalRead(PIN_DOUT) == HIGH) && ((millis() - startTime) < 1000))
    ;  //wait for data conversion ready

  if ((millis() - startTime) > 1000)  //or time out...
  {
    Serial.println("weight error");
  }
  RawWeight = 0;
  // pulse the clock pin 24 times to read the data
  for (char i = 0; i < 24; i++) {
    digitalWrite(PIN_CLOCK, HIGH);
    delayMicroseconds(1);
    RawWeight = RawWeight << 1;
    if (digitalRead(PIN_DOUT) == HIGH) RawWeight++;
    digitalWrite(PIN_CLOCK, LOW);
  }
  // set the channel and the gain factor (A 128) for the next reading using the clock pin (one pulse)
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(1);
  RawWeight = RawWeight ^ 0x800000;
  digitalWrite(PIN_CLOCK, LOW);
  AverageWeight += RawWeight;

  //digitalWrite(PIN_CLOCK, HIGH);    //to enter into power saving mode
  CurrentRawWeight = AverageWeight;
#ifdef xRAW_WEIGHT_DEBUG
  Serial.print("Raw weight : \t");
  Serial.println(RawWeight);
#endif
#ifdef RAW_WEIGHT_DEBUG
  Serial.print("Raw average weight : ");
  Serial.println(CurrentRawWeight);
#endif
}
#endif

#ifdef HAS_BACK_SCALE
void GetRawWeight2(void) {
  unsigned long RawWeight2;
  // wait for the chip to become ready
  long startTime;
  //delay(5000);             //let the HX711 warm up
  AverageWeight2 = 0;

  startTime = millis();

  while ((digitalRead(PIN_DOUT2) == HIGH) && ((millis() - startTime) < 1000))
    ;  //wait for data conversion ready

  if ((millis() - startTime) > 1000)  //or time out...
  {
    Serial.println("weight error");
  }
  RawWeight2 = 0;
  // pulse the clock pin 24 times to read the data
  for (char i = 0; i < 24; i++) {
    digitalWrite(PIN_CLOCK2, HIGH);
    delayMicroseconds(1);
    //nanoDelay(100);
    RawWeight2 = RawWeight2 << 1;
    if (digitalRead(PIN_DOUT2) == HIGH) RawWeight2++;
    digitalWrite(PIN_CLOCK2, LOW);
  }
  // set the channel and the gain factor (A 128) for the next reading using the clock pin (one pulse)
  digitalWrite(PIN_CLOCK2, HIGH);
  delayMicroseconds(1);

  RawWeight2 = RawWeight2 ^ 0x800000;
  digitalWrite(PIN_CLOCK2, LOW);

  AverageWeight2 += RawWeight2;

  //digitalWrite(PIN_CLOCK, HIGH);    //to enter into power saving mode
  CurrentRawWeight2 = AverageWeight2;
#ifdef xRAW_WEIGHT_DEBUG
  Serial.print("Raw weight2 : \t");
  Serial.println(RawWeight2);
#endif
#ifdef RAW_WEIGHT_DEBUG
  Serial.print("Raw average weight2 : ");
  Serial.println(CurrentRawWeight2);
#endif
}
#endif


float mapf(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
  float result;
  result = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  return result;
}

void getCalibZero() {
  //front load cell init
#ifdef HAS_FRONT_SCALE
  pinMode(PIN_CLOCK, OUTPUT);  // initialize digital pin 4 as an output.(clock)
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);        //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);  //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);      // initialize digital pin 5 as an input.(data Out)

  GetRawWeight();
  calibZero = CurrentRawWeight;  // during boot the motor doesn't spin = no thrust on the propeller


//preferences.putLong("calibZero", calibZero); // no need to store it: computed during each boot and used during loop
#ifdef DEBUG_CAL0
  Serial.print("front scale calibZero raw value = ");
  Serial.println(calibZero);
#endif
#endif  //HAS_FRONT_SCALE

  //back load cell init
#ifdef HAS_BACK_SCALE
  pinMode(PIN_CLOCK2, OUTPUT);  // initialize digital pin 4 as an output.(clock)
  digitalWrite(PIN_CLOCK2, HIGH);
  delayMicroseconds(100);         //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK2, LOW);  //exit sleep mode*/
  pinMode(PIN_DOUT2, INPUT);      // initialize digital pin 5 as an input.(data Out)

  GetRawWeight2();
  calibZero2 = CurrentRawWeight2;  // during boot the motor doesn't spin = no thrust on the torque load cell
//preferences.putLong("calibZero2", calibZero2); // no need to store it: computed during each boot and used during loop
#ifdef DEBUG_CAL0
  Serial.print("back scale calibZero2 raw value = ");
  Serial.println(calibZero2);
#endif
#endif  //HAS_BACK_SCALE
}

// -------------------------------------------------------------------------------------------------
//                                        customise the menus below
// -------------------------------------------------------------------------------------------------

// Useful commands:
//      void reWaitKeypress(20000);         = wait for the button to be pressed on the rotary encoder (timeout in 20 seconds if not)
//      chooseFromList(8, "TestList", q);   = choose from the list of 8 items in a string array 'q'
//      enterValue("Testval", 15, 0, 30);   = enter a value between 0 and 30 (with a starting value of 15)


// Available Menus
#ifdef HAS_ENCODER
// main menu
void Main_Menu() {
  menuTitle = "Menu";              // set the menu title
  setMenu(0, "");                  // clear all menu items
  setMenu(0, "tare");              // choose from a list
  setMenu(1, "set L, L1, L2...");  // enter a value
  setMenu(2, "calibration");       // display a message
  setMenu(3, "return");            // display a message
}

// menu 2
void menu2() {
  menuTitle = "calibration";
  setMenu(0, "");
  setMenu(0, "tare");
  setMenu(1, "calib front scale");
  setMenu(2, "calib back scale");
  setMenu(3, "set calib weight");
  setMenu(4, "return");
}


// -------------------------------------------------------------------------------------------------
// menu action procedures
//   check if menu item has been selected with:  if (menuTitle == "<menu name>" && menuItemClicked==<item number 1-4>)

void menuItemActions() {

  if (menuItemClicked == 100) return;  // if no menu item has been clicked exit function

  //  --------------------- Main Menu Actions ------------------
  if (menuTitle == "Menu" && menuItemClicked == 0) {
    menuItemClicked = 100;
    Serial.println("Menu: tare selected");
    // display a message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 20);
    display.print("tare scales");
    display.display();
    reWaitKeypress(2000);  // wait for key press on rotary encoder
    getCalibZero();
  }
  if (menuTitle == "Menu" && menuItemClicked == 1) {
    menuItemClicked = 100;  // flag that the button press has been actioned (the menu stops and waits until this)
    String q[] = { "set L", "set CoG", "set L1", "set L2" };
    int tres = chooseFromList(4, "TestList", q);
    Serial.println("Menu: item " + String(tres) + " chosen from list");
    switch (tres) {
      case 0:
        Serial.println("set L");
        menuItemClicked = 100;
        tres = enterValue("Lever length L (mm)", length, 1, 0, 500);  // enter a value (title, start value, step size, low limit, high limit)
        Serial.println("L set = " + String(tres));
        length = tres;
        preferences.putInt("length", length);
        break;
      case 1:
        Serial.println("target CoG");
        tres = enterValue("target CoG (mm)", targetCoG, 1, 0, 400);  // enter a value (title, start value, step size, low limit, high limit)
        Serial.println("CoG set = " + String(tres));
        targetCoG = tres;
        preferences.putInt("CoG", targetCoG);
        break;
      case 2:
        Serial.println("set L1");
        tres = enterValue("dist LE to nose", L1, 1, 0, 300);  // enter a value (title, start value, step size, low limit, high limit)
        Serial.println("L1 set = " + String(tres));
        L1 = tres;
        preferences.putInt("L1", L1);
        break;
      case 3:
        Serial.println("set L2");
        tres = enterValue("dist LE to tail", L2, 1, 0, 1000);  // enter a value (title, start value, step size, low limit, high limit)
        Serial.println("L2 set = " + String(tres));
        L2 = tres;
        preferences.putInt("L2", L2);
        break;
      default:
        // statements
        break;
    }
  }

  if (menuTitle == "Menu" && menuItemClicked == 2) {
    menuItemClicked = 100;
    Serial.println("Menu: Menu 2 selected");
    menu2();  // show a different menu
  }



  if (menuTitle == "Menu" && menuItemClicked == 3) {
    menuItemClicked = 100;
    Serial.println("Menu: exit selected");
    // Display splash screen on OLED

    menuTitle = "";  //(will close the menu)
  }
  //  --------------------- Menu 2 Actions ---------------------

  if (menuTitle == "calibration" && menuItemClicked == 0) {
    menuItemClicked = 100;
    Serial.println("Menu: tare selected");
    // display a message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 20);
    display.print("tare scales");
    display.display();
    reWaitKeypress(2000);  // wait for key press on rotary encoder
    getCalibZero();
  }

  if (menuTitle == "calibration" && menuItemClicked == 1) {
    menuItemClicked = 100;
    Serial.println("Menu: tare selected");
    // display a message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 20);
    display.print("calib front scale");
    display.display();
    reWaitKeypress(2000);  // wait for key press on rotary encoder
    GetRawWeight();        //HX711 sensor

    calib = calibZero - CurrentRawWeight;
    if (calib == 0) calib = 130968;
    Serial.print("calibration front scale... ");
    Serial.println(calib);
    Serial.print(" for weight (g) ");
    Serial.println(calibWeight);

    preferences.putLong("calib", calib);
  }
  if (menuTitle == "calibration" && menuItemClicked == 2) {
    menuItemClicked = 100;
    Serial.println("Menu: tare selected");
    // display a message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(20, 20);
    display.print("calib back scale");
    display.display();
    reWaitKeypress(2000);  // wait for key press on rotary encoder
    GetRawWeight2();       //HX711 sensor backscale
    calib2 = calibZero2 - CurrentRawWeight2;
    if (calib2 == 0) calib2 = 130968;
    Serial.print("calibration back scale... ");
    Serial.println(calib2);
    Serial.print(" for weight (g) ");
    Serial.println(calibWeight2);
    preferences.putLong("calib2", calib2);
  }
  if (menuTitle == "calibration" && menuItemClicked == 3) {
    menuItemClicked = 100;
    Serial.println("Menu: set calib weight selected");
     calibWeight = enterValue("calib weight (g)", calibWeight, 1, 100, 10000);  // enter a value (title, start value, step size, low limit, high limit)
        Serial.println("calib weight set = " + String(calibWeight));
        preferences.putInt("calibWeight", calibWeight);
  }
  if (menuTitle == "calibration" && menuItemClicked == 4) {
    menuItemClicked = 100;
    Serial.println("Menu: exit selected");
    // Display splash screen on OLED

    Main_Menu();
  }
}
// -------------------------------------------------------------------------------------------------
//                                        customise the menus above
// -------------------------------------------------------------------------------------------------

//  -------------------------------------------------------------------------------------------
//  ------------------------------------- menu procedures -------------------------------------
//  -------------------------------------------------------------------------------------------


// set menu item
// pass: new menu items number, name         (blank iname clears all entries)

void setMenu(byte inum, String iname) {
  if (inum >= menuMax) return;  // invalid number
  if (iname == "") {            // clear all menu items
    for (int i = 0; i < menuMax; i++) menuOption[i] = "";
    menuCount = 0;  // move highlight to top menu item
  } else {
    menuOption[inum] = iname;
    menuItemClicked = 100;  // set item selected flag as none
  }
}


//  --------------------------------------

// display menu on oled
void staticMenu() {
  display.clearDisplay();
  // title
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(lineSpace1, 0);
  display.print(menuTitle);
  display.drawLine(0, lineSpace1, display.width(), lineSpace1, WHITE);

  // menu options
  int i = 0;
  while (i < menuMax && menuOption[i] != "") {                     // if menu item is not blank display it
    if (i == menuItemClicked) display.setTextColor(BLACK, WHITE);  // if this item has been clicked
    else display.setTextColor(WHITE, BLACK);
    display.setCursor(10, 18 + (i * lineSpace1));
    display.print(menuOption[i]);
    i++;
  }

  // highlighted item if none yet clicked
  if (menuItemClicked == 100) {
    display.setCursor(2, 18 + (menuCount * lineSpace1));
    display.print(">");
  }

  display.display();  // update display
}

#endif
//  --------------------------------------


// rotary encoder button
//    returns 1 if the button status has changed since last time
#ifdef HAS_ENCODER
bool menuCheck() {


  // oled menu action on button press
  if (isEncoderButtonClicked()) {  // if button is now pressed

    if (menuTitle == "") {
      Main_Menu();  // start the menu displaying - see menuItemActions() to alter the menus
      return 0;
    }
    if (menuItemClicked != 100 || menuTitle == "") return 1;  // menu item already selected or there is no live menu
    Serial.println(menuTitle);

#ifdef E_DEBUG
    Serial.println("menu '" + menuTitle + "' item " + String(menuCount) + " selected");
#endif
    menuItemClicked = menuCount;  // set item selected flag
  }

  return 1;
}



// wait for key press or turn on rotary encoder
//    pass timeout in ms
void reWaitKeypress(int timeout) {
  uint32_t tTimer = millis();  // log time
                               // wait for button to be released
                               /*  while ((!isEncoderButtonClicked()) && (millis() - tTimer < timeout)) {  // wait for button release
    yield();                                                                                   // service any web page requests
    delay(20);
  }*/
  // clear rotary encoder position counter
  enc.resetTicks();
  // wait for button to be pressed or encoder to be turned
  while ((!isEncoderButtonClicked()) && (enc.getTicks() == 0) && (millis() - tTimer < timeout)) {
    yield();  // service any web page requests
    delay(20);
  }
  exitMenu();  // close menu
}


//  --------------------------------------


// handle menu item selection

void menuItemSelection() {
  if (enc.getTicks() >= itemTrigger) {
    enc.resetTicks();
    if (menuCount + 1 < menuMax) menuCount++;      // if not past max menu items move
    if (menuOption[menuCount] == "") menuCount--;  // if menu item is blank move back
  }
  if (enc.getTicks() <= -itemTrigger) {
    enc.resetTicks();
    if (menuCount > 0) menuCount--;
  }
}


// enter a value using the rotary encoder
//   pass Value title, starting value, step size, low limit , high limit
//   returns the chosen value
int enterValue(String title, int start, int stepSize, int low, int high) {
  uint32_t tTimer = millis();  // log time of start of function
                               // display title
  display.clearDisplay();
  display.setTextSize(1);  // if title is longer than 8 chars make text smaller
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(title);
  display.display();  // update display
  int tvalue = start;

  tTimer = millis();
  while (!isEncoderButtonClicked()) {  // && (millis() - tTimer < (OLEDDisplayTimeout * 1000))) {  // while button is not pressed and still within time limit
                                       // encoder0Pos is updated via the interrupt procedure
    tvalue = getValue(start, low, high);
    display.setTextSize(3);
    const int textPos = 27;                                                      // height of number on display
    display.fillRect(0, textPos, SCREEN_WIDTH, SCREEN_HEIGHT - textPos, BLACK);  // clear bottom half of display (128x64)
    display.setCursor(0, textPos);
    display.print(tvalue);
    // bar graph at bottom of display
    int tmag = map(tvalue, low, high, 0, SCREEN_WIDTH);
    display.fillRect(0, SCREEN_HEIGHT - 10, tmag, 10, WHITE);
    display.display();  // update display
    yield();            // service any web page requests
  }
  exitMenu();  // close menu
  return tvalue;
}

int getValue(int start, int low, int high) {
  int tvalue;
  return tvalue = constrain(start + enc.getTicks(), low, high);
}

//  --------------------------------------


// choose from list using rotary encoder
//  pass the number of items in list (max 8), list title, list of options in a string array

int chooseFromList(byte noOfElements, String listTitle, String list[]) {

  const byte noList = 10;      // max number of items to list
  uint32_t tTimer = millis();  // log time of start of function
  int highlightedItem = 0;     // which item in list is highlighted
  int xpos, ypos;

  // display title
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(10, 0);
  display.print(listTitle);
  display.drawLine(0, lineSpace1, display.width(), lineSpace1, WHITE);

  // scroll through list
  while (((isEncoderButtonClicked())) && (millis() - tTimer < (OLEDDisplayTimeout * 1000))) delay(5);  // wait for button release
  tTimer = millis();
  while ((!isEncoderButtonClicked()) && (millis() - tTimer < (OLEDDisplayTimeout * 1000))) {  // while button is not pressed and still within time limit
    if (enc.getTicks() >= itemTrigger) {                                                      // encoder0Pos is updated via the interrupt procedure
      highlightedItem++;
      enc.resetTicks();
      delay(100);
      tTimer = millis();
    }
    if (enc.getTicks() <= -itemTrigger) {
      highlightedItem--;
      enc.resetTicks();
      delay(100);
      tTimer = millis();
    }
    // value limits
    if (highlightedItem > noOfElements - 1) highlightedItem = noOfElements - 1;
    if (highlightedItem < 0) highlightedItem = 0;
    // display the list
    for (int i = 0; i < noOfElements; i++) {
      if (i < (noList / 2)) {
        xpos = 0;
        ypos = lineSpace1 * (i + 1) + 7;
      } else {
        xpos = display.width() / 2;
        ypos = lineSpace1 * (i - ((noList / 2) - 1)) + 7;
      }
      display.setCursor(xpos, ypos);
      if (i == highlightedItem) display.setTextColor(BLACK, WHITE);
      else display.setTextColor(WHITE, BLACK);
      display.print(list[i]);
    }
    display.display();  // update display
    yield();            // service any web page requests
  }

  // if it timed out set selection to cancel (i.e. item 0)
  if (millis() - tTimer >= (OLEDDisplayTimeout * 1000)) highlightedItem = 0;

  //  // wait for button to be released (up to 1 second)
  //    tTimer = millis();                         // log time
  //    while ( (digitalRead(encoder0Press) == LOW) && (millis() - tTimer < 1000) ) {
  //      yield();        // service any web page requests
  //      delay(20);
  //    }

  exitMenu();  // close menu

  return highlightedItem;
}


//  --------------------------------------


// close the menus and return to sleep mode

void exitMenu() {
  menuCount = 3;
  enc.resetTicks();
}

#endif

// ---------------------------------------------- end ----------------------------------------------
