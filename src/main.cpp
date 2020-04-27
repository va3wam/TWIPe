/*************************************************************************************************************************************
 * @file main.cpp
 * @author va3wam
 * @brief Run tests to determine the fastest times we can use on TWIPe for OLED and MQTT updates of data
 * @version 0.0.1
 * @date 2020-04-25
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ----------------------------------------------------------------------------------------------------------------
 * 0.0.2   2020-04-27 Special trimmed down build that just doe IMU output to the serial port
 * 0.0.1   2020-03-13 Program created
 *************************************************************************************************************************************/
// TODO Add ability to get commands sent from MQTT broker and  process them. Start with toggle of MQTT balance telemetry
// TODO Add boot sequence that 1) checks Flash for config, or 2) asks MQTT for config, or 3) Uses default values in include file
// TODO Add MQTT topic which is updated at boot up
// TODO Make motor logic, use small up time. Time between raising edge to affect speed
// TODO Add metadata MQTT topic and update every 30 seconds 
#include <Arduino.h> // Arduino Core for ESP32 from https://github.com/espressif/arduino-esp32. Comes with Platform.io
#include <WiFi.h> // Required to connect to WiFi network. Comes with Platform.io
#include <Adafruit_MQTT.h> // https://github.com/adafruit/Adafruit_MQTT_Library
#include <Adafruit_MQTT_Client.h> // https://github.com/adafruit/Adafruit_MQTT_Library
#include <I2Cdev.h> // For MPU6050 - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/I2Cdev/I2Cdev.h
#include <MPU6050_6Axis_MotionApps_V6_12.h> // For  MPU6050 - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps_V6_12.h
#include <Wire.h> // Required for I2C communication. Comes with Platform.io
#include <SSD1306.h> // For OLED - https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <huzzah32_pins.h> // Defines GPIO pins for Adafruit Huzzah32 dev board
#include <i2c_metadata.h> // Defines all I2C related information incluing device addresses, bus pins and bus speeds
#include <known_networks.h> // Defines Access points and passwords that the robot can scan for and connect to

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Create debug macros that mirror the standard c++ print functions. Use the pre-processor variable 
/// DEBUG to toggle debug messages on and off. NOte that all debug messages will get processed without the 
/// need of clunky if statements. When DEBUG is false the macros simply do nothing.  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG true // Comment/uncomment this line to turn debug tracing on/off
#define DMP_TRACE false // Set to TRUE or FALSE to toggle DMP memory read/write activity
#if DEBUG == true
    #define AMDP_PRINT(x) Serial.print(x)
    #define AMDP_PRINTF(x, y) Serial.print(x, y)
    #define AMDP_PRINTLN(x) Serial.println(x)
    #define AMDP_PRINTLNF(x, y) Serial.println(x, y)
#else // Map macros to "do nothing" commands so that when is not TRUE these commands do nothing
    #define AMDP_PRINT(x)
    #define AMDP_PRINTF(x, y)
    #define AMDP_PRINTLN(x)
    #define AMDP_PRINTLNF(x, y)
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define OLED constants, classes and global variables 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SSD1306 display(rightOLED_I2C_ADD, gp_I2C_LCD_SDA, gp_I2C_LCD_SCL);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define LED constants, classes and global variables 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool blinkState = false;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define MPU6050 constants, classes and global variables 
/// @note We are using Yaw/Pitch/Roll which suggers from gimble lock
/// @link http://en.wikipedia.org/wiki/Gimbal_lock) 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MPU6050 mpu; // GY521 default I2C address
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
/// @brief orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
// Offset values for MPU6050 readings. See readme.md file for details 
int16_t XGyroOffset; // Gyroscope x axis (Roll)
int16_t YGyroOffset; // Gyroscope y axis (Pitch)
int16_t ZGyroOffset; // Gyroscope z axis (Yaw)
int16_t XAccelOffset; // Accelerometer x axis
int16_t YAccelOffset; // Accelerometer y axis
int16_t ZAccelOffset; // Accelerometer z axis

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief MPU6050 DMP firmware interrupt constants, classes and global variables 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
portMUX_TYPE dmpMUX = portMUX_INITIALIZER_UNLOCKED; // Syncronize variables between the DMP data ready ISR and loop()
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Loop timer interrupt constants, classes and global variables 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
hw_timer_t * timer0Pointer = NULL; // Define pointer used to set up a hardware timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // Syncronize variables between the timer ISR and loop()
#define TIMER0 1 // Use the first of the four timers available to keep track of counters that control events in loop()
#define TIMER0_PRESCALER 80 // Prescaling value to set to 1Mhz (80Mhz clock / 80 prescaler = 1Mhz)
#define TIMER0_CNT_UP true // Set TIMER0 to count up
#define TIMER0_INT_EDGE true // Define the TIMER0 intterrupt condition as edge
#define TIMER0_RELOAD true // Specifies that the counter will reload, allowing the interrupt to fire more than once
#define TIMER0_INT_CNT 20 // Number that TIMER0 counts up before triggering an interrupt

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Variables used to store current WiFi network information for the robot
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const char* mySSID = "NOTHING";
const char* myPassword =  "NOTHING";
String myMACaddress; // MAC address of this SOC. Used to uniquely identify this robot 
String myIPAddress; // IP address of the SOC.
String myAccessPoint; // WiFi Access Point that we managed to connected to 
String myHostName; // Name by which we are known by the Access Point
String myHostNameSuffix = "Twipe"; // Suffix to add to WiFi host name for this robot 
WiFiClient client; // Create an ESP8266 WiFiClient class to connect to the MQTT server

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define MQTT constants, classes and global variables. 
/// @note MQTT broker used for testing this was Mosquitto running on a Raspberry Pi
/// @note sends balance telemetry data to <device name><telemetry/balance> 
/// @note listens for commands on <device name><commands>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MQTT_BROKER_IP "192.168.2.93" // Need to make this a fixed IP address
#define MQTT_BROKER_PORT 1883 // Use 8883 for SSL
#define MQTT_USERNAME "NULL" // Not used at this time. To do: secure MQTT broker
#define MQTT_KEY "NULL" // Not used at this time. To do: secure MQTT broker
#define MQTT_IN_CMD "/commands" // Topic branch for incoming remote commands
#define MQTT_TEL_BAL "/telemetry/balance" // Topic tree for outgoing balance telemetry data
#define MQTT_READ_TIMEOUT 500 // Number of tries to read a command from the broker per loop (see Adafruit_MQTT_Client::readPacket)
String MQTT_Incoming_Commands; // Topic tree we subscribe to to get remote commands
String MQTT_Telemetry_Balance; // Topic tree we publish balance telemetry to

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define FreeRTOS task variables and objects
/// @note With FreeRTOS, low priority numbers denote low priority tasks. Max priority is 25 (configMAX_PRIORITIES)
/// @note configUSE_TIME_SLICING is set to 1 in FreeRTOSConfig.h so tasks of equal priority will share the available processing time 
///       using a time sliced round robin scheduling scheme. 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define CORE_1 1 // Core 1 of the ESP32 SOC
//#define MQTT_TASK_STACK_SIZE 4000 // Number of bytes of memry set aside to run MQTT management task 
//#define MQTT_TASK_PRIORITY 1 // Priority that the MQTT management task runs at. 
//TaskHandle_t MQTTmanagerTaskHandle; // Handle for task that handles MQTT communication 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize MQTT broker connection and both SUB and PUB topics
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Adafruit_MQTT_Client mqtt(&client, MQTT_BROKER_IP, MQTT_BROKER_PORT, MQTT_USERNAME, MQTT_KEY); // Connect to MQTT broker
Adafruit_MQTT_Publish topicTelemetryBalance = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME MQTT_TEL_BAL, MQTT_QOS_1); // Outgoing balance data
Adafruit_MQTT_Subscribe topicRemoteCommands = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME MQTT_IN_CMD, MQTT_QOS_1); // Incoming commands
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME MQTT_IN_CMD, MQTT_QOS_1);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Global control variables.  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define tmrIMU 200 // Milliseconds to wait between reading data to IMU over I2C
#define tmrOLED 200 // Milliseconds to wait between sending data to OLED over I2C
#define tmrMQTT 200 // Milliseconds to wait between sending data to MQTT broker over WiFi  
#define tmrMETADATA 2000 // Milliseconds to wait between sending data to serial port
#define tmrLED 1000 / 2 // Milliseconds to wait between flashes of LED (turn on / off twice in this time)
uint32_t cntLoop = 0; // Track how many times loop() has iterated
int goIMU = 0; // Target time for next read of IMU data 
int goOLED = 0; // Target time for next OLED update
int goMQTT = 0; // Target time for next MQTT broker update
int goMETADATA = 0; // Target time for next serial port
int goLED = 0; // Target time for next toggle of LED
int MQTT_publish_balance_fail_cnt = 0; // Track how many times pubishing balance data returns a fail code
int MQTT_publish_balance_success_cnt = 0; // Track how many times pubishing balance data returns a success code
int DMP_FIFO_data_missing_cnt = 0; // Track how many times the FIFO pin goes high but the buffer is empty 
int DMP_FIFO_data_exists_cnt = 0; // Track how many times the FIFO pin goes high and there is data in the buffer 
boolean sendMetaDataToMQTT = false; // Used to decide if metadata about the code shoud go to MQTT broker or serial port

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define flags that are used to track what devices/functions are verified working after start up. Initilize false.  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boolean leftOLED_detected = false;
boolean rightOLED_detected = false;
boolean LCD_detected = false;
boolean MPU6050_detected = false;
boolean wifi_detected = false;

/*************************************************************************************************************************************
 * @brief Interrupt Service Routine (ISR) that runs when the DMP firmware on the MPU6050 raises its interrupt pin indicating that it
 * has data in its FIFO buffer ready to be read over I2C 
 * @note This function is not placed in IRAM
 *************************************************************************************************************************************/
// TODO Understand the use or IRAM
void dmpDataReady() 
{
  portENTER_CRITICAL(&dmpMUX); // Prevent loop() from updating variable while we are changing it
  mpuInterrupt = true; // Flag the fact that there is data ready to be read
  portEXIT_CRITICAL(&dmpMUX); // Allow loop() access to variable again
} //dmpDataReady()

/*************************************************************************************************************************************
 * @brief Strips the colons off the MAC address of this device
 * @return String
 *************************************************************************************************************************************/
String formatMAC()
{
  String mac;
  Serial.println("<formatMAC> Removing colons from MAC address");
  mac = WiFi.macAddress(); // Get MAC address of this SOC
  mac.remove(2,1); // Remove first colon from MAC address
  mac.remove(4,1); // Remove second colon from MAC address
  mac.remove(6,1); // Remove third colon from MAC address
  mac.remove(8,1); // Remove forth colon from MAC address
  mac.remove(10,1); // Remove fifth colon from MAC address
  Serial.print("<formatMAC> Formatted MAC address without colons = ");
  Serial.println(mac);
  return mac;
} //formatMAC()

/***********************************************************************************************************
 * @brief This function returns a String version of the local IP address
 ***********************************************************************************************************/
String ipToString(IPAddress ip)
{
    Serial.println("<ipToString> Converting IP address to String.");
    String s="";
    for (int i=0; i<4; i++)
    {
      s += i  ? "." + String(ip[i]) : String(ip[i]);
    } //for
    Serial.print("<ipToString> IP Address = ");
    Serial.println(s);
    return s;
} //ipToString()

/***********************************************************************************************************
 This function returns a String version of the local IP address.
 ***********************************************************************************************************/
void connectToNetwork() 
{
  int ConnectCount = 20; // Maximum number of AP connection attemts 
  int ConnectAttempts = 0; // Number of AP connection attempts made
  String tmpHostNameVar; // Hold WiFi host name created in this function

  Serial.println("<connectToNetwork> Attempt to connect to an Access Point");
  Serial.print("<connectToNetwork> Try connecting to AP = ");
  Serial.println(mySSID);
  WiFi.begin(mySSID, myPassword);
  while ((WiFi.status() != WL_CONNECTED) && (ConnectCount > 0)) 
  {
    delay(1000);
    Serial.print("<connectToNetwork> Establishing connection to WiFi. Connect attempt count down = ");
    Serial.println(ConnectCount);
    ConnectCount--;
    ConnectAttempts++;
  } //while
  
  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("<connectToNetwork> Connection to network FAILED");
    wifi_detected = false; // Set status of WiFi connection to FALSE 
  } //if
  else
  {
    myIPAddress = ipToString(WiFi.localIP());
//    myMACaddress = formatMAC();
    myAccessPoint = WiFi.SSID();
    tmpHostNameVar = myHostNameSuffix + myMACaddress;
    WiFi.setHostname((char*)tmpHostNameVar.c_str());
    myHostName = WiFi.getHostname();
    wifi_detected = true; // Set status of WiFi connection to TRUE 
    Serial.print("<connectToNetwork> Connection to network SUCCESSFUL after this many attempts: ");
    Serial.println(ConnectAttempts);
    Serial.println("<connectToNetwork> Network information is as follows..."); 
    Serial.print("<connectToNetwork> - Access Point Robot is connected to = ");
    Serial.println(myAccessPoint);
    Serial.print("<connectToNetwork> - Robot Network Host Name = ");
    Serial.println(myHostName);
    Serial.print("<connectToNetwork> - Robot IP Address = ");
    Serial.println(myIPAddress);
    Serial.print("<connectToNetwork> - Robot MAC Address = ");
    Serial.println(myMACaddress);
  } //else
} //connectToNetwork() 

/***********************************************************************************************************
 * @brief This function translates the type of encryption that an Access Point (AP) advertises (an an ENUM) 
 * and returns a more human readable description of what that encryption method is.
 ***********************************************************************************************************/
String translateEncryptionType(wifi_auth_mode_t encryptionType) 
{
  switch (encryptionType) 
  {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
    default:
      return "UNKNOWN";
  } //switch
} //translateEncryptionType()

/***********************************************************************************************************
 * @brief This function scans the WiFi spectrum looking for Access Points (AP). It selects the AP with the 
 * strongest signal which is included in the known network list.
 ***********************************************************************************************************/
void scanNetworks() 
{
  int numberOfNetworks = WiFi.scanNetworks(); // Used to track how many APs are detected by the scan
  int StrongestSignal = -127; // Used to find the strongest signal. Set as low as possible to start
  int SSIDIndex = 0; // Contains the SSID index number from the known list of APs
  bool APknown; // Flag to indicate if the current AP appears in the known AP list

  Serial.println("<scanNetworks> Scanning for WiFi Access Points.");
  Serial.print("<scanNetworks> Number of networks found: ");
  Serial.println(numberOfNetworks);
  
  // Loop through all detected APs
  for (int i = 0; i < numberOfNetworks; i++) 
  {
    APknown = false;
    Serial.print("<scanNetworks> Network name: ");
    Serial.println(WiFi.SSID(i));
    Serial.print("<scanNetworks> Signal strength: ");
    Serial.println(WiFi.RSSI(i));
    Serial.print("<scanNetworks> MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));
    Serial.print("<scanNetworks> Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    
    // Scan table of known APs to see if the current AP is known to us 
    for (int j =0; j< numKnownAPs; j++)
    {
      // If the current scanned AP appears in the known AP list note the index value and flag found
      if(WiFi.SSID(i) == SSID[j])
      {
        APknown = true;
        SSIDIndex = j;
        Serial.println("<scanNetworks> This is a known network");
        wifi_detected = true;
      } //if
    } //for

    // If the current AP is known and has a stronger signal than the others that have been checked 
    // then store it in the variables that will be used to connect to the AP later
    if((APknown == true) && (WiFi.SSID(i).toInt() > StrongestSignal))
    {
      mySSID = SSID[SSIDIndex].c_str();
      myPassword = Password[SSIDIndex].c_str();
      StrongestSignal = WiFi.SSID(i).toInt();
      Serial.println("<scanNetworks> This is the strongest signal so far");
    } //if
    Serial.println("<scanNetworks> -----------------------");
  } //for

  Serial.print("<scanNetworks> Best SSID candidate = ");
  Serial.println(mySSID);
} //scanNetworks()

/*************************************************************************************************************************************
 * @brief Connect and reconnect to the MQTT broker as needed
 * @note Should be called in the loop function and it will take care if connecting
 *************************************************************************************************************************************/
void MQTT_connect() 
{
  int8_t ret;
  if(mqtt.connected()) // Exit this function if already connected to MQTT broker
  {
    return;
  } //if
  Serial.println("<MQTT_connect> Not connected to MQTT broker. Attempting to connect/reconnect now");
  uint8_t retries = 3;
  while((ret = mqtt.connect()) != 0) // connect() will return 0 for connected
  { 
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("<MQTT_connect> Retrying MQTT connection in 5 seconds");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if(retries == 0) 
    {         
      Serial.println("<MQTT_connect> Cannot connect to MQTT broker. Halting execution");
      while(1); // loop forever
    } //if
  } //while
  Serial.println("<MQTT_connect> Connected/reconnected to MQTT broker");
} //MQTT_connect()

/*************************************************************************************************************************************
 * @brief Print leading zeros for a binary number
 * @note Code taken from Peter H Anderson example
 * @arg int8_t v = number to dispay
 * @arg int8_t num_places = number of bits to display (normally a multile of 8)
 *************************************************************************************************************************************/
void printBinary(byte v, int8_t num_places)
{
  int8_t mask=0, n;
  for (n=1; n<=num_places; n++)
  {
      mask = (mask << 1) | 0x0001;
  } //for
  v = v & mask;  // truncate v to specified number of places
  while(num_places)
  {
      if (v & (0x0001 << (num_places-1)))
      {
            Serial.print("1");
      } //if
      else
      {
            Serial.print("0");
      } //else
      --num_places;
      if(((num_places%4) == 0) && (num_places != 0))
      {
          Serial.print("_");
      } //if
  } //while
} //printBinary()

/*************************************************************************************************************************************
 * @brief Retrieve message from MQTT broker 
 *************************************************************************************************************************************/
void cmdCallback(char *data, uint16_t len) 
{
  Serial.print("<cmdCallback> Command recieved = ");
  Serial.println(data);
} //cmdCallback()

/*************************************************************************************************************************************
 * @brief Retrieve message from MQTT broker 
 *************************************************************************************************************************************/
// TODO Put this into its own thread so that it can run without interfering with telemetry posting and balancing logic
void getMQTTcmd()
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Wait for a fixed period of time for any incoming remote commands
  /// @note Spend most of the loop time in this while loop to ensure that sent commands do nt get dropped
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(MQTT_READ_TIMEOUT))) 
  {
    if(subscription == &topicRemoteCommands) 
    {
      Serial.print(F("<MQTTmanager> Got: "));
      Serial.println((char *)topicRemoteCommands.lastread);
    } //if
  } //while
} //getMQTTcmd()

/*************************************************************************************************************************************
 * @brief Send message to MQTT broker 
 * @param angle Angle of robot lean in eulers. To convert to degrees use this formula: degrees = angle * 180 / PI
 *************************************************************************************************************************************/
void updateMQTT(float angle)
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Maintain MQTT broker connection
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MQTT_connect(); // Ensure the connection to the MQTT server is alive
  String tmp = String(millis()) + "," + String(angle * 180 / PI);
  char msg[15];
  tmp.toCharArray(msg,sizeof(tmp));
  //if(! topicTelemetryBalance.publish(angle * 180 / PI))  // Publish uint32_t to MQTT broker 
  boolean rtn = topicTelemetryBalance.publish(msg); // Publish string out to MQTT broker
  if(!rtn)  // Check for fail error code
  {
    MQTT_publish_balance_fail_cnt++; // Track how many times publishing balance data returns a fail code.
  } //if
  else // Put counter here to  track successful posts
  {
    MQTT_publish_balance_success_cnt++; // Track how many times publishing balance data returns a success code.
  } //else
  goMQTT = millis() + tmrMQTT; // Reset MQTT broker update target time  
} //UpdateMQTT()

/*************************************************************************************************************************************
 * @brief Send updated metadata about the running of the code.
 * @param sendToMQTT 
 * # Table of Metadata tracked for TWIPe Robot Code
 * | Item | Data Point                                                                                                  |
 * |:-----------------------|:-------------------------------------------------------------------------------------------|
 * | Time Stamp             | Time ESP32 has been runing in milli seconds |
 * | Balance telemetry send | Success count  |
 * | Balance telemetry send | Failure count  |
 * | MPU6050 DMO FIFO read  | Success count  |
 * | MPU6050 DMO FIFO read  | Failure count  |
 *************************************************************************************************************************************/
void updateMetaData(float yaw, float pitch, float roll)
{
 // display Euler angles in degrees in console
  Serial.print("<updateMetaData> ypr\t");
  Serial.print(yaw * 180 / PI); // Change M_PI to PI
  Serial.print("\t");
  Serial.print(pitch * 180 / PI); // Change M_PI to PI
  Serial.print("\t");
  Serial.println(roll * 180 / PI); // Change M_PI to PI
} // updateMetaData()

/*************************************************************************************************************************************
 * @brief Update OLED dipsplay 
 * @param angle Angle of robot lean in eulers. To convert to degrees use this formula: degrees = angle * 180 / PI
 * @note We are working with Yaw/Pitch/Roll data (and only using pitch). Other options include euler, quaternion, raw acceleration, 
 * raw gyro, linear acceleration and gravity. See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
 *************************************************************************************************************************************/
void updateOLED(float angle)
{
  display.clear();        
  display.drawString(64, 20, String(angle * 180 / PI));
  display.display();  
  goOLED = millis() + tmrOLED; // Reset OLED update target time
} //UpdateOLED()

/*************************************************************************************************************************************
 * @brief Set up a WiFi connection
 *************************************************************************************************************************************/
void setupWiFi()
{
  scanNetworks();
  connectToNetwork(); 
} // setupWiFi()

/*************************************************************************************************************************************
 * @brief Set up communication with an MQTT broker
 *************************************************************************************************************************************/
void setupMQTT()
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief If there is no WiFi connection quit this function immediatey
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(!wifi_detected)
  {
      Serial.print("<setupMQTT> No WiFi,  cannot communicate with MQTT broker.");
      return;
  } //if
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Set up topic tree for sending balancing telemetry data
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MQTT_Telemetry_Balance = myHostName + MQTT_TEL_BAL; // Balance telemetry tree we publish to
  topicTelemetryBalance.topic = MQTT_Telemetry_Balance.c_str(); // Set balance topic name to use MAC address and defined branch path 
  Serial.print("<setupMQTT> Balance Telemetry will be sent to ");Serial.println(MQTT_Telemetry_Balance);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Set up topic tree for incoming remote commands
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  MQTT_Incoming_Commands = myHostName + MQTT_IN_CMD; // Topic tree we subscribe to to get remote commands
  topicRemoteCommands.topic = MQTT_Incoming_Commands.c_str(); // Set remote topic name to use MAC address and defined branch path
  Serial.print("<setupMQTT> Subscribe to command feed "); Serial.println(MQTT_Incoming_Commands);
//  topicRemoteCommands.setCallback(cmdCallback); // Set up callback for incoming remote commands
  mqtt.subscribe(&topicRemoteCommands); // Subscribe to MQTT topic containing incoming remote commands
  Serial.print("<setupMQTT> MQTT keep alive set to "); Serial.print(MQTT_CONN_KEEPALIVE); Serial.println(" seconds");
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Create seperate FreeRTOS thread and run a function that will manage MQTT communication
  /// @details The ESP32 has 2 Tensilica LX6 cores. In order to specify which core the MQTTManager code runs on we must make use of 
  /// FreeRTOS and ESP32 IDF primitives. Both the SETUP() and LOOP()) are executed with a priority of 1 and in core 1. Priorities can 
  /// range from 0 to 25, where 0 is the lowest priority Core can be 0 or 1
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  xTaskCreatePinnedToCore(MQTTcommand, // First argument is function to run
                          "MQTTcommand", // Second argument is the name of task (for debug purposes)
                          MQTT_TASK_STACK_SIZE, // Third argument is the stack size in words. 
                          NULL, // Fourth argument is arguments (parameter) passed to the function
                          MQTT_TASK_PRIORITY, // Fifth argument is the FreeRTOS priority of the task
                          &MQTTmanagerTaskHandle, // Sixth argument is the task handle
                          CORE_1); // Seventh argument specifies which of the two CPU cores to pin this task to  
*/
} //setupMQTT()

/*************************************************************************************************************************************
 * @brief Set up the LED that is flashed by loop()
 *************************************************************************************************************************************/
void setupMainLoopLED()
{
  Serial.println(F("<setupMainLoopLED> Enable LED pin"));
  pinMode(gp_SWC_LED, OUTPUT); // configure LED for output
} //setupMainLoopLED()

/*************************************************************************************************************************************
 * @brief Set up the OLED 
 *************************************************************************************************************************************/
void setupOLED()
{
  Serial.println(F("<setupOLED> Initialize OLED"));
  display.init();
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 20, "My Demo"); //64,22  
  display.display();
  Serial.println(F("<setupOLED> Initialization of OLED complete"));
} //setupOLED()

/*************************************************************************************************************************************
 * @brief Set up the MPU6050 using DMP firmware and interrupts
 *************************************************************************************************************************************/
void setupIMU()
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief initialize device
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("<setupIMU> Initializing MPU6050..."));
  mpu.initialize();
  pinMode(gp_IMU_INT, INPUT);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief verify connection
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("<setupIMU> Testing MPU6050 connection..."));
  bool tmp = mpu.testConnection();
  if(tmp == true)
  {
    Serial.println("<setupIMU> MPU6050 connection successful");
  } //if
  else
  {
    Serial.println("<setupIMU> MPU6050 connection failed. Halting boot up");
    while(1);
  } //else
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief load and configure the DMP
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("<setupIMU> Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) 
  {
    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(XGyroOffset);
    mpu.setYGyroOffset(YGyroOffset);
    mpu.setZGyroOffset(ZGyroOffset);
    mpu.setXAccelOffset(XAccelOffset);
    mpu.setYAccelOffset(YAccelOffset);
    mpu.setZAccelOffset(ZAccelOffset);
    // Generate offsets and calibrate MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("<setupIMU> Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.print(F("<setupIMU> Enabling DMP FIFO data ready GPIO pin "));
    Serial.println(digitalPinToInterrupt(gp_IMU_INT));
    Serial.println(F("<setupIMU> Attaching inetrrupt pin to dmpDataReady function"));
    attachInterrupt(digitalPinToInterrupt(gp_IMU_INT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.print("<setupIMU> MPU initialization status = ");
    Serial.println(mpuIntStatus);
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print("<setupIMU> packetSize = ");
    Serial.println(packetSize);
    // Initial read of DMP FIFO  
  } //if
  else 
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// @brief Initialization faied. 
    /// @note If it's going to break, usually the code will be 1
    /// @note 1 = initial memory load failed
    /// @note 2 = DMP configuration updates failed
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Serial.print(F("<setupIMU> DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while(1); // loop forever thus halting boot up
  } //else
} //setupIMU()

/*************************************************************************************************************************************
 * @brief Construct a new cfg by MAC object
 *************************************************************************************************************************************/
void cfg_by_MAC()
{
    myMACaddress = formatMAC();
    if(myMACaddress == "BCDDC2F7D6D5")    // This is Andrew's bot
    {
      Serial.println("<cfg_by_MAC> Setting up MAC BCDDC2F7D6D5 configuration");
      XGyroOffset = 135;
      YGyroOffset = -9;
      ZGyroOffset = -85;
      XAccelOffset = -3396;
      YAccelOffset = 830;
      ZAccelOffset = 1890;      
    } //if
    else if(myMACaddress == "B4E62D9EA8F9")    // This is Doug's bot
    {
      Serial.println("<cfg_by_MAC> Setting up MAC BCDDC2F7D6D5 configuration");
      XGyroOffset = 60;
      YGyroOffset = -10;
      ZGyroOffset = -72;
      XAccelOffset = -2070;
      YAccelOffset = -70;
      ZAccelOffset = 1641;      
    } //else if
    else
    {
      Serial.println("<cfg_by_MAC> MAC not recognized. Setting up generic configuration");
      XGyroOffset = 135;
      YGyroOffset = -9;
      ZGyroOffset = -85;
      XAccelOffset = -3396;
      YAccelOffset = 830;
      ZAccelOffset = 1890;      
    } //else
} //cfg_by_MAC()

/*************************************************************************************************************************************
 * @brief Flash AMBER LED on fron panel button
 *************************************************************************************************************************************/
void updateLED()
{
  blinkState = !blinkState;
  digitalWrite(gp_SWC_LED, blinkState);
  goLED = millis() + tmrLED; // Reset LED flashing counter
} // updateLED()

/*************************************************************************************************************************************
 * @brief Retrieve DMP FIFO data
 *************************************************************************************************************************************/
// TODO learn about the three different dmpGet commands used here. Do we need them all? What do they do? an we call only 1? 
void readIMU()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Check to see if there is any data in the DMP FIFO buffer. 
  {  
    mpu.dmpGetQuaternion(&q, fifoBuffer); // Get the latest packet of Quaternion data
    mpu.dmpGetGravity(&gravity, &q); // Get the latest packet of gravity data 
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get the latest packet of Euler angles 
    DMP_FIFO_data_exists_cnt++; // Track how many times the FIFO pin goes high and the buffer has data in it
  } //if
  else // If DMP pin goes high but there is no data in the FIFO buffer then something weird happend
  {
    DMP_FIFO_data_missing_cnt++; // Track how many times the FIFO pin goes high but the buffer is empty  
  } //else
  goIMU = millis() + tmrIMU; // Reset IMU update counter  
} // updateLED()

/*************************************************************************************************************************************
 * @brief Standard set up routine for Arduino programs
 *************************************************************************************************************************************/
void setup() 
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Initialize two wire bus known as Wire() with the pins and transmission speed required to communicate with the MPU6050
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(gp_I2C_IMU_SDA,gp_I2C_IMU_SCL,I2C_bus1_speed);  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Set up serial communication
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200); // Open a serial connection at 115200bps
  while (! Serial); // Wait for Serial port to be ready
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Announce start of startup
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("<setup> Start of setup"));
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Set up communication and configuration
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  cfg_by_MAC(); // Use the devices MAC address to make specific configuration settings
  setupMainLoopLED(); // Set up the LED that the loop() flashes
//  setupWiFi(); // Set up WiFi communication
//  setupOLED(); // Setup OLED communication
//  setupMQTT(); // Set up MQTT communication
  setupIMU(); // Set up IMU communication
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Announce end of startup
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("<setup> End of setup"));
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Reset all timing counters
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  goOLED = millis() + tmrOLED; // Reset OLED update counter
  goMQTT = millis() + tmrMQTT; // Reset MQTT broker update counter
  goLED = millis() + tmrLED; // Reset LED flashing counter
  goIMU = millis() + tmrIMU; // Reset IMU update counter
  goMETADATA = millis() + tmrMETADATA; // Reset IMU update counter
  cntLoop = 0; // Loop counter does not interact with timer ISR so no muxing is required 
  mqtt.subscribe(&onoffbutton);
} //setup()

/*************************************************************************************************************************************
 * @brief Standard looping routine for Arduino programs
 *************************************************************************************************************************************/
void loop() 
{
  cntLoop ++; // Increment loop() counter
  if((millis() >= goIMU) && mpuInterrupt == true) 
  {
    readIMU(); // Update the OLED with data
    if(millis() >= goMETADATA) updateMetaData(ypr[0], ypr[1], ypr[2]); // Send data to serial terminal
  } //if
  delay(100);
} //loop()