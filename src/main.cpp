/*************************************************************************************************************************************
 * @file main.cpp
 * @author va3wam
 * @brief Gathers balacing telemetry data from MPU6050 via DMP firmware and sends it to MQTT broker
 * @version 0.1.10
 * @date 2020-03-13
 * @copyright Copyright (c) 2020
 * @note We are working with Yaw/Pitch/Roll data (and only using pitch). Other options include euler, quaternion, raw acceleration, 
 * raw gyro, linear acceleration and gravity. See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ----------------------------------------------------------------------------------------------------------------
 * 0.1.10  2020-04-22 Added logic to manage configuration details by MAC address. So far only the MPU6050 offset values use this. Also
 *                    changed DEBUG_PRINT 
 * 0.1.9   2020-04-21 Added new default offset values after running https://github.com/va3wam/TWIPeCalibrate  
 * 0.1.8   2020-04-21 Moved GPIO pin definitions to huzzah32_pins.h and I2C meta data to i2c_metadata.h. Also added DEBUG flag and 
 *                    associated DEBUG_PRINT macros.
 * 0.1.7   2020-04-15 Swapped GPIO pins to match TWIPe SB7D PCB wiring
 * 0.1.6   2020-03-22 Fixed version comment
 * 0.1.5   2020-03-22 Cleaned up comments. Renamed AIO variables to MQTT since we are not using the Adafruit AIO server
 * 0.1.4   2020-03-22 Nested timer flag, DMP data ready flag and buffer read results to all DMP FIFO data reads in order to stabalize
 *                    IMU data read logic.  Also moved IMU calibration logic inside status check of IMU to avoid attempting to 
 *                    configure the IMU while it is in a bad state. Cleaned up comments and removed old commented out code. Removed
 *                    MQTT manager function (replaced with updateMQTT). Added counters to track erros with MQTT writing and DMP 
 *                    reading. Added counter resets at the end of setup(). Replaced delay(100) at bottom of loop() with millis()
 *                    logic to make iterations consistent.
 * 0.1.3   2020-03-20 Added timer ISR logic to manage event timing in loop()
 * 0.1.2   2020-03-18 Removed FreeRTOS task and broke setup into more steps. Everything works but is too slow and seems to hang
 * 0.1.1   2020-03-14 Added FreeRTOS task pegged to core 0 that controls MQTT communication
 * 0.1.0   2020-03-13 Program created
 *************************************************************************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO Add in command listen logic, possibly in its own FreeRTOS thread
// TODO Stabalize code to stop it from randomly hanging. Possibly use a local FIFO buffer to do this
// TODO in UpdateMQTT() change data published from loop counter to MPU6050 pitch values
// TODO Once an MQTT send fails they all seem to fail without recovering
// TODO Add secure connection to MQTT broker. It is currenty unsecure
// TODO program locks up at random times even if just reading MPU6050 and dislaying on OLED (no MQTT). 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    #define AMDP(x) Serial.print(x)
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
SSD1306 display(rightOLED_I2C_ADD, I2C_bus0_SDA, I2C_bus0_SCL);

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
/// @brief Define WiFi constants, classes and global variables 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
Adafruit_MQTT_Publish topicTelemetryBalance = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME MQTT_TEL_BAL,MQTT_QOS_1); // Outgoing balance data
Adafruit_MQTT_Subscribe topicRemoteCommands = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME MQTT_IN_CMD,MQTT_QOS_1); // Incoming commands

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Global control variables.  
/// @note Different events require different timing. To keep things coordinated timer values have been selected that are multiples of 
/// each other while being consistent with the performance seen for each device during testing. These values are as follows:
/// Item     | Freq (Hz) | Counter   
/// :------- | :-------: | :-------:
/// Loop     | 500       | 100
/// MPU6050  | 100       | 500
/// OLED     | 250       | 200
/// MQTT     | 20        | 2500
/// LED      | 1         | 50000
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define tarIMU 500 // MPU6050 is to be updated at 100Hz (TIMER0 count of 500) 
#define tarOLED 200 // OLED is to be updated at 250Hz (TIMER0 count of 200)
#define tarMQTT 2500 // MQTT broker is to be updated at 20Hz (TIMER0 count of 2500)  
#define tarLED 50000 // LED is to be updated at 1Hz (TIMER0 count of 50000) 
uint32_t cntLoop = 0; // Track how many times loop() has iterated
volatile int cntIMU = 0; // Track how many times the MPU6050 DMP firmware has signalled that a packet it ready to be read 
volatile int cntOLED = 0; // Track how many times loop() has iterated since last OLED update
volatile int cntMQTT = 0; // Track how many times loop() has iterated since last post to MQTT broker
volatile int cntLED = 0; // Track how many times loop() has iterated since last LED value inversion
int MQTT_publish_balance_fail_cnt = 0; // Track how many times pubishing balance data returns a fail code
int DMP_FIFO_data_missing_cnt = 0; // Track how many times the FIFO pin  goes high but the buffer is empty 
unsigned long previousMillis = 0; // Tracks the last time loop() ended. Used to control iteration speed of loop()
#define LOOP_INTERVAL 100 // Sets the target time for how long loop() should take before iterating (in millis())
unsigned long currentMillis = 0; // Tracks the current value of millis(). Used to control iteration speed of loop()
unsigned long loopDelay = 0; // Track how many delay loops occur at the end of loop() to ensure consistent iterations.

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
 * @brief Interrupt Service Routine (ISR) that runs when the timer counter reaches its configured target
 * @note This function definition includes the IRAM_ATTR attribute in order for the compiler to place the code in IRAM. When this is 
 * done the function should only call functions also placed in IRAM
 * TODO Understand the use or IRAM
 *************************************************************************************************************************************/
void IRAM_ATTR onTimer() 
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Increment counters used to time when different actions will take place
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  portENTER_CRITICAL_ISR(&timerMux); // Prevent loop() from updating variable while we are changing it
  cntOLED ++; // Increment OLED update counter
  cntMQTT ++; // Increment MQTT broker update counter
  cntLED ++; // Increment LED flashing counter
  cntIMU++; // Increment IMU update counter
  portEXIT_CRITICAL_ISR(&timerMux); // Allow loop() access to variable again
} //onTimer()

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
 *************************************************************************************************************************************/
void UpdateMQTT(float yaw, float pitch, float roll)
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Maintain MQTT broker connection
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MQTT_connect(); // Ensure the connection to the MQTT server is alive
//  if(! topicTelemetryBalance.publish(pitch))  // Publish pitch value to MQTT broker
  if(! topicTelemetryBalance.publish(cntLoop))  // Publish cntLoop to MQTT broker.  For testing only
  {
    MQTT_publish_balance_fail_cnt++; // Track how many times publishing balance data returns a fail code. Program can continue.
  } //if
} //UpdateMQTT()

/*************************************************************************************************************************************
 * @brief Update serial port terminal with balance data
 *************************************************************************************************************************************/
void UpdateSerial(float yaw, float pitch, float roll)
{
  // display Euler angles in degrees in console
  Serial.print("<UpdateSerial> ypr\t");
  Serial.print(yaw * 180 / PI); // Change M_PI to PI
  Serial.print("\t");
  Serial.print(pitch * 180 / PI); // Change M_PI to PI
  Serial.print("\t");
  Serial.println(roll * 180 / PI); // Change M_PI to PI
} // UpdateSerial()

/*************************************************************************************************************************************
 * @brief Update OLED dipsplay 
 * @note We are working with Yaw/Pitch/Roll data (and only using pitch). Other options include euler, quaternion, raw acceleration, 
 * raw gyro, linear acceleration and gravity. See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
 *************************************************************************************************************************************/
void UpdateOLED(float yaw, float pitch, float roll)
{
  display.clear();        
  display.drawString(64, 20, String(roll * 180 / PI));
  display.display();        
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
 * @details Set up the timer that sets flags used to control events in loop()
 *************************************************************************************************************************************/
void setupLoopTimer()
{
  Serial.println(F("<setupLoopTimer> Configure TIMER0 to call ISR onTimer()"));
  timer0Pointer = timerBegin(TIMER0, TIMER0_PRESCALER, TIMER0_CNT_UP); // Define pointer for timer0
  timerAttachInterrupt(timer0Pointer, &onTimer, TIMER0_INT_EDGE); // Bind TIMER0 to the handling function onTimer(), interrupt on EDGE
  timerAlarmWrite(timer0Pointer, TIMER0_INT_CNT, TIMER0_RELOAD); // Specify counter value that will generae a TIMER0 interrupt
  timerAlarmEnable(timer0Pointer); // Enable TIMER0 
} //setupLoopTimer()

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
    if(myMACaddress == "BCDDC2F7D6D5")
    {
      Serial.println("<cfg_by_MAC> Setting up MAC BCDDC2F7D6D5 configuration");
      XGyroOffset = 135;
      YGyroOffset = -9;
      ZGyroOffset = -85;
      XAccelOffset = -3396;
      YAccelOffset = 830;
      ZAccelOffset = 1890;      
    } //if
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
 * @brief Standard set up routine for Arduino programs
 *************************************************************************************************************************************/
void setup() 
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Initialize two wire bus known as Wire() with the pins and transmission speed required to communicate with the MPU6050
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin(I2C_bus1_SDA,I2C_bus1_SCL,I2C_bus1_speed);  
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
  setupMainLoopLED(); // Set up the LED tht the loop() flashes
  setupWiFi(); // Set up WiFi communication
  setupOLED(); // Setup OLED communication
  /// TODO Add this back to use MQTT
//  setupMQTT(); // Set up MQTT communication
  setupIMU(); // Set up IMU communication
  setupLoopTimer(); // Set up the timer that controls loop() events
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Announce end of startup
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("<setup> End of setup"));
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Reset all timing counters
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  portENTER_CRITICAL_ISR(&timerMux); // Prevent loop() from updating variable while we are changing it
  cntOLED = 0; // Reset OLED update counter
  cntMQTT = 0; // Reset MQTT broker update counter
  cntLED = 0; // Reset LED flashing counter
  cntIMU = 0; // Reset IMU update counter
  portEXIT_CRITICAL_ISR(&timerMux); // Allow loop() access to variable again
  cntLoop = 0; // Loop counter does not interact with timer ISR so no muxing is required 
  currentMillis = millis(); // Initialize counter for timing loop() 
  previousMillis = currentMillis; // Set new target time for loop() iteration
} //setup()

/*************************************************************************************************************************************
 * @brief Standard looping routine for Arduino programs
 *************************************************************************************************************************************/
void loop() 
{
  cntLoop ++; // Increment loop() counter
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Display the current counter values for each event
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
  Serial.print("<loop> LED/OLED/MQTT/IMU/LOOP | MQTT fails/DMP fails/Loop Delay Cnt\t"); 
  Serial.print(cntLED); Serial.print("\t");
  Serial.print(cntOLED); Serial.print("\t");
  Serial.print(cntMQTT); Serial.print("\t");
  Serial.print(cntIMU); Serial.print("\t");
  Serial.print(cntLoop); Serial.print("\t | \t"); 
  Serial.print(MQTT_publish_balance_fail_cnt); Serial.print("\t");
  Serial.print(DMP_FIFO_data_missing_cnt); Serial.print("\t");
  Serial.println(loopDelay);
*/
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Retrieve DMP FIFO data when the flag indicates that the MPU6050 DMP has raised its interrupt pin on the GY521 board
  /// @note In order to read data from the DMP FIFO the following conditions must be met: 1) The IMU counter must have hit its 
  /// target value, 2) the DMP interrupt flag must be set true, and 3) the DMP FIFO buffer must have data in it
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // TODO learn about the three different dmpGet commands used here. Do we need them all? What do they do? an we call only 1?
  // TODO Make counter to go into else logic to count how often the buffer is found empty. Why pin high but buffer empty?
  // TODO Sometimes locks up right away. Why?
  if(cntIMU >= tarIMU) // Check IMU timer 
  {
    cntIMU = 0; // Reset IMU counter
    if(mpuInterrupt == true) // DMP interrupt flag went high and has not been reset
    {
      portENTER_CRITICAL(&dmpMUX); // Start coordinate ISR and main loop
      mpuInterrupt = false; // Reset DMP data ready flag
      portEXIT_CRITICAL(&dmpMUX); // End coordinate ISR and loop    
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Check to see if there is any data in the DMP FIFO buffer. 
      {  
        mpu.dmpGetQuaternion(&q, fifoBuffer); // Get the latest packet of Quaternion data
        mpu.dmpGetGravity(&gravity, &q); // Get the latest packet of gravity data 
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get the latest packet of Euler angles 
      } //if
      else // If DMP pin goes high but there is no data in the FIFO buffer then something weird happend
      {
        DMP_FIFO_data_missing_cnt++; // Track how many times the FIFO pin  goes high but the buffer is empty  
      } //else
    } //if
  } //if
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Update serial with balance data when the time is right
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  UpdateSerial(ypr[0], ypr[1], ypr[2]); // Send data to serial terminal
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Update OLED with balance data when the time is right
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(cntOLED >= tarOLED) // Check OLED counter to see if it is time to send new data to the OLED
  {
    portENTER_CRITICAL_ISR(&timerMux); // Prevent ISR from updating variable while we are changing it
    cntOLED = 0; // Reset OLED update counter  
    portEXIT_CRITICAL_ISR(&timerMux); // Allow ISR access to variable again  
    UpdateOLED(ypr[0], ypr[1], ypr[2]); // Send data to OLED
  } //if
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Update MQTT when the time is right
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// TODO Add this back to use MQTT 
  /* 
  if(cntMQTT >= tarMQTT)
  {
    portENTER_CRITICAL_ISR(&timerMux); // Prevent ISR from updating variable while we are changing it
    cntMQTT = 0; // Reset MQTT update counter  
    portEXIT_CRITICAL_ISR(&timerMux); // Allow ISR access to variable again  
    UpdateMQTT(ypr[0], ypr[1], ypr[2]); // Send data to MQTT broker    
  } //if
  */
/*
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Ping MQTT broker to keep connection open. Only required of period between communicatins is more than KEEPALIVE value
  /// @note Keepalive value shows in console at startup. Default is 5 min
  /// if/when other MQTT communication is required.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // TODO Add timer to track timeout of this and PING as needed.  Note required for balancing telemetry but may be needed
  if(! mqtt.ping()) 
  {
    mqtt.disconnect();
  } //if  
*/
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Update MQTT when the time is right
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // TODO Handle this in seperate FreeRTOS thread, not here in the loop() function
//  getMQTTcmd(); // Get command from MQTT broker 
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Blink LED to indicate activity
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(cntLED >= tarLED)
  {
    portENTER_CRITICAL_ISR(&timerMux); // Prevent ISR from updating variable while we are changing it
    cntLED = 0; // Reset LED flashing counter  
    portEXIT_CRITICAL_ISR(&timerMux); // Allow ISR access to variable again  
    blinkState = !blinkState;
    digitalWrite(gp_SWC_LED, blinkState);
  } //if
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief Ensure that loop() always takes the same amount of time
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  loopDelay = 0; // reset counter 
  while(currentMillis - previousMillis <= LOOP_INTERVAL) // If loop() has not taken long enough
  {
    loopDelay++;
    currentMillis = millis(); // Increment current time variable until the full  intervaalhas passed
  } // while
  previousMillis = currentMillis; // Set new target time to now 
} //loop()