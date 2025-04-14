
#include <WiFiClient.h>
#include <TelnetStream.h>
#include "driver/adc.h"
#include <esp_wifi.h>
#include <esp_bt.h>


int nbTimeout = 0;
int timeOut = 0;
int nbAlarm = 0;
long bootTime;

//Json
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson


String displayStatus = "Initializing";
String res;

//OLED
#define OLED
#ifdef OLED
#define OLED_SCL_PIN 5
#define OLED_SDA_PIN 17
#include "SSD1306.h"
SSD1306 display(0x3c, OLED_SDA_PIN, OLED_SCL_PIN);  // for 0.96" 128x64 LCD display ; i2c ADDR & SDA, SCL
#endif
String displayDebug;
long lastOled;





//sensors selection
#define HAS_FRONT_SCALE  
#define HAS_BACK_SCALE   

bool hasAcquiredSensors = false;

//wifi option
#define HAS_AP  //will only show an access point
#ifdef HAS_AP
// Replace with your network credentials
const char* APssid = "JP RC_CoG_finder";
const char* APpassword = "";
#endif

float temperature = 0;
int smooth = 10;  //acquire smooth*values for each ADC

#define PIN_CONF 13   //CONF pin on the board. Used for wifi manager
#define PIN_SW 16     //switch pin for rotary encoder
#define PIN_ENC_A 15  //rotary encoder A pin
#define PIN_ENC_B 4   //rotary encoder B pin


//front scale
#define PIN_CLOCK 14  //output to generate clock on Hx711
#define PIN_DOUT 12   //input Dout from Hx711

//back scale
#define PIN_CLOCK2 32  //output to generate clock on Hx711_2
#define PIN_DOUT2 33   //input Dout from Hx711_2

long calibZero = 0;      //No load front scale Output
long calib = 130968;     //sensor output - calibZero for Weight calibration --> will be auto calibrated later
int calibWeight = 1000;  //weight at which calibration is done --> expressed in grams.
float AverageWeight = 0;
float CurrentRawWeight = 0;
int acquisitionFrequency;

long calibZero2 = 0;      //No load back scale sensor Output
long calib2 = 130968;     //sensor output - calibZero2 for Weight calibration --> will be auto calibrated later
int calibWeight2 = 1000;  //weight at which calibration is done --> expressed in grams.
float AverageWeight2 = 0;
float CurrentRawWeight2 = 0;

int length;  //scale lever length
float CoG, targetCoG;   //Center of Gravity
int L1 ;     //nose to LE distance
int L2;      //tail to LE distance

#define LED_PIN 22



//Preferences
#include <Preferences.h>
Preferences preferences;

//touchpad
touch_pad_t touchPin;
int threshold = 40;  //Threshold value for touchpads pins

void callback() {
  //placeholder callback function
}
boolean TouchWake = false;



//WifiManager
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager

//flag for saving data
bool shouldSaveConfig = false;
bool touch3detected = false;  //touch3 used to launch WifiManager (hold it while reseting)
bool touch2detected = false;  //touch2 used to calibrate loadcell with known weight (hold it while reseting)

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


String ssid = "";
String password = "";
boolean hasWifiCredentials = false;


//#define W_DEBUG     //debug Wifi and firebase
//#define G_DEBUG     //debug GCM serveur
//#define DEBUG_OUT
//#define xDEBUG
//#define xxDEBUG
#define UDP_DEBUG
//#define DEBUG
//#define TEST
#define PREFERENCES_DEBUG
//#define DEBUG_TELNET
//#define RAW_WEIGHT_DEBUG
//#define DEBUG_W


//UDP --------------
unsigned int localPort = 5000;  // local port to listen on
char packetBuffer[64];          //buffer to hold incoming packet
char AndroidConnected = 0;
long Timeout;
String device = "RC_CoG_finder";
String theMAC = "";

WiFiUDP Udp;
long LastUDPnotification;
//end UDP-----------


void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);

  Serial.println("***************");
  Serial.println("program started");
  Serial.println("***************");
  Serial.println(" ");


  //Preferences
  preferences.begin("RC_CoG_finder", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only

  calibWeight = preferences.getInt("calibWeight", 500);  //by default calibration of load cell is done with 500g load attached to the front scale
  calib = preferences.getLong("calib", -222444);         //

  calibWeight2 = preferences.getInt("calibWeight2", 500);  //by default calibration of load cell is done with 500g load attached to the back scale
  calib2 = preferences.getLong("calib2", -222444);         // value for  ~50g (surepeesed when calibration done on the android App)

  length  = preferences.getInt("length", 200);  //by default calibration of lever length is 200mm
  L1 = preferences.getInt("L1", 100);  //by default calibration of L1 is 100mm
  L2 = preferences.getInt("L2", 800);  //by default calibration of L2 is 800mm
  targetCoG = preferences.getInt("CoG", 80);  //by default to 80mm

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

#ifdef OLED
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "initialization");
  display.display();
#endif

#ifdef HAS_AP
  // setup Wi-Fi network with SSID and password
  Serial.printf("Setting AP (Access Point)… '%s'\n", APssid);
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(APssid, APpassword);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  Serial.print("UDP started ");
#ifdef OLED
  display.drawString(0, 20, "AP : JP RC_CoGfinder");
  display.display();
#endif

#else
#ifdef OLED
  display.drawString(0, 20, "Connect to Wifi");
  display.display();
#endif
  if (ftouchRead(T3) < threshold) touch3detected = true;  //detect touchpad for CONFIG_PIN
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

  if (touch3detected)  //then launch WifiManager : touch the Touch3 pin GPIO15 , reset the ESP32 then release the touch pin a Wifi Access point will pop up
  {
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

  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;

  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 20000)) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;
#endif

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

  Serial.println(" ");
  Serial.println("start monitoring sensors : \n");
#ifdef OLED
  display.drawString(0, 30, "sensor calibration");
  display.display();
#endif



  bootTime = millis();

  getCalibZero();

#ifdef DEBUG_TELNET
  TelnetStream.print("start");
#endif

  LastUDPnotification = millis();
}

//***********************************************************************************************************************
void loop() {
  // esp_task_wdt_reset();  //reset the watchdog
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

//AverageWeight = 600;
//AverageWeight2 = 300;
if ((AverageWeight + AverageWeight2) != 0) CoG = AverageWeight2 * length / (AverageWeight + AverageWeight2);
 else CoG = .01; 


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




  //send sensors data as fast as possible (else uncomment the following line)
  if (((millis() - LastUDPnotification) > 300))  // send UDP message to Android App (no need to be connected, a simple notification, send and forget)
  {

    res = "{\"F\": " + String(AverageWeight) + ",\"B\": " + String(AverageWeight2) + ",\"L\": " + String(length)+ ",\"L1\": " + String(L1) + ",\"L2\": " + String(L2)  + ",\"C\": " + String(CoG)+ ",\"TC\": " + String(targetCoG)  + "}";
    LastUDPnotification = millis();
#ifdef DEBUG_TELNET
    TelnetStream.println(res);
#endif
    sendUDP(res);  //try to send via UDP
#ifdef OLED
    displayLCD();  //and update OLED display
#endif
  }


  if (((millis() - timeOut) > 6000) && (AndroidConnected == 1)) {
    AndroidConnected = 0;
    Serial.println("lost connection with Android phone, switch to Manual mode");
  }
}  //end of Loop



#ifdef OLED
void displayLCD(void)  //refresh the LCD screen
{
  if ((millis() - lastOled) > 50)  // to keep display readible
  {
    display.clear();
    //display.drawString(0, 0, "current : " + (String)current + " A");
    //display.drawString(0, 10, "voltage : " + (String)Vin + " V");
    display.drawString(0, 20, "thrust  : " + (String)AverageWeight + " g");
    // display.drawString(0, 30, "RPM  : " + (String)RPM);
    //display.drawString(0, 40, "freq : " + (String)acquisitionFrequency + " Hz");
    display.drawString(0, 40, "torque  : " + (String)AverageWeight2 + " g");
    // display.drawString(0, 50, "throttle : " + (String)throttle);
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
      if (Cmd == "CalF")  //front scale calibration
      {
#ifdef HAS_FRONT_SCALE
        String value = doc["value"];
        calibWeight = value.toInt();
        GetRawWeight();  //HX711 sensor

        calib = calibZero - CurrentRawWeight;
        if (calib ==0) calib =130968;
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
        GetRawWeight2();  //HX711_2 sensor

        calib2 = calibZero2 - CurrentRawWeight2;
        if (calib2 ==0) calib2 =130968;
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
      } 
       else if (Cmd == "SetL1")  //lever Length calibration
      {

        String value = doc["value"];
        L1 = value.toInt();
        Serial.print("set L1 Length... ");
        Serial.print(L1);
        Serial.println(" mm ");

        preferences.putInt("L1", L1);
      } 
       else if (Cmd == "SetL2")  //lever Length calibration
      {

        String value = doc["value"];
        L2 = value.toInt();
        Serial.print("set L2... ");
        Serial.print(L2);
        Serial.println(" mm ");

        preferences.putInt("L2", L2);
      } 
      else if (Cmd == "SetCoG")  //lever Length calibration
      {

        String value = doc["value"];
        targetCoG = value.toInt();
        Serial.print("set target CoG... ");
        Serial.print(targetCoG );
        Serial.println(" mm ");

        preferences.putInt("CoG", targetCoG);
      } else if (Cmd == "Cal0")  //lever Length calibration
      {


        Serial.println("tare scales... ");
        getCalibZero();
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
#ifdef DEBUG_W
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
#ifdef DEBUG_W
  Serial.print("back scale calibZero2 raw value = ");
  Serial.println(calibZero2);
#endif
#endif  //HAS_BACK_SCALE
}
