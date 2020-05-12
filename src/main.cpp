/*************************************************************************************************************************************
 * @file main.cpp
 * @author va3wam
 * @brief Run tests to determine the fastest times we can use on TWIPe for OLED and MQTT updates of data
 * @version 0.0.3
 * @date 2020-04-25
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ----------------------------------------------------------------------------------------------------------------
 * 0.0.3   2020-05-12 Added manual motor control via MQTT
 * 0.0.2   2020-04-27 Special trimmed down build that just doe IMU output to the serial port
 * 0.0.1   2020-03-13 Program created
 *************************************************************************************************************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TODO Add boot sequence that 1) checks Flash for config, or 2) asks MQTT for config, or 3) Uses default values in include file
// TODO Add MQTT topic which is updated at boot up
// TODO Fix bug where sometimes MQTT commands do not terminate and the command goes forever
/// @brief Arduino libraries
#include <Arduino.h> // Arduino Core for ESP32 from https://github.com/espressif/arduino-esp32. Comes with Platform.io
#include <WiFi.h> // Required to connect to WiFi network. Comes with Platform.io
#include <I2Cdev.h> // For MPU6050 - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/I2Cdev/I2Cdev.h
#include <MPU6050_6Axis_MotionApps_V6_12.h> // For  MPU6050 - https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps_V6_12.h
#include <Wire.h> // Required for I2C communication. Comes with Platform.io
#include <SSD1306.h> // For OLED - https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <huzzah32_pins.h> // Defines GPIO pins for Adafruit Huzzah32 dev board
#include <i2c_metadata.h> // Defines all I2C related information incluing device addresses, bus pins and bus speeds
#include <known_networks.h> // Defines Access points and passwords that the robot can scan for and connect to
#include <AsyncMqttClient.h> // https://github.com/marvinroger/async-mqtt-client

/// @brief FreeRTOS libraries  
#include "freertos/FreeRTOS.h" // Required for threads that control wifi and mqtt connections
#include "freertos/timers.h" // Required for xTimerCreate function used for controlling wifi and mqtt connections
#include <freertos/event_groups.h> // Required to use the FreeRTOS function xEventGroupSetBits. Used for motor driver control
#include <freertos/queue.h> // Required to use FreeRTOS the function uxQueueMessagesWaiting. Used for motor driver control

/// @brief Precompiler directives for debug output 
#define DEBUG true // Turn debug tracing on/off
#define DMP_TRACE false // Set to TRUE or FALSE to toggle DMP memory read/write activity

/// @brief Create debug macros that mirror the standard c++ print functions. Use the pre-processor variable 
#if DEBUG == true
    #define AMDP_PRINT(x) Serial.print(x)
    #define AMDP_PRINTLN(x) Serial.println(x)
#else // Map macros to "do nothing" commands so that when is not TRUE these commands do nothing
    #define AMDP_PRINT(x)
    #define AMDP_PRINTLN(x)
#endif

/// @brief Define which core the Arduino environment is running on
#if CONFIG_FREERTOS_UNICORE // If this is an SOC with only 1 core
  #define ARDUINO_RUNNING_CORE 0 // Arduino is running on that one core
#else // If this is an SOC with more than one core (2 is the ony other option at ths point)
  #define ARDUINO_RUNNING_CORE 1 // Arduino is running on the second core
#endif

/// @brief Define robot specific global parameters 
int16_t XGyroOffset; // Gyroscope x axis (Roll)
int16_t YGyroOffset; // Gyroscope y axis (Pitch)
int16_t ZGyroOffset; // Gyroscope z axis (Yaw)
int16_t XAccelOffset; // Accelerometer x axis
int16_t YAccelOffset; // Accelerometer y axis
int16_t ZAccelOffset; // Accelerometer z axis
float wheelDiameter; // Diameter of wheel 
float wheelCircumference; // Diameter x pi
int stepsPerRevolution; // How many steps our motors need to take to do one complete revolution
float distancePerStep; // How far our robot moves per step
float heightCOM = 5; // How far it is from the ground to the Centre Of Mass of the robot

/// @brief Define OLED constants, classes and global variables 
SSD1306 rightOLED(rightOLED_I2C_ADD, gp_I2C_LCD_SDA, gp_I2C_LCD_SCL);

/// @brief Define LED constants, classes and global variables 
bool blinkState = false;

/// @brief Define MPU6050 constants, classes and global variables 
/// @note We are using Yaw/Pitch/Roll which suggers from gimble lock
/// @link http://en.wikipedia.org/wiki/Gimbal_lock) 
MPU6050 mpu; // GY521 default I2C address
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
portMUX_TYPE dmpMUX = portMUX_INITIALIZER_UNLOCKED; // Syncronize variables between the DMP data ready ISR and loop()
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

/// @brief Define global WiFi network information 
const char* mySSID = "NOTHING";
const char* myPassword =  "NOTHING";
String myMACaddress; // MAC address of this SOC. Used to uniquely identify this robot 
String myIPAddress; // IP address of the SOC.
String myAccessPoint; // WiFi Access Point that we managed to connected to 
String myHostName; // Name by which we are known by the Access Point
String myHostNameSuffix = "Twipe"; // Suffix to add to WiFi host name for this robot 
WiFiClient client; // Create an ESP32 WiFiClient class to connect to the MQTT server
TimerHandle_t wifiReconnectTimer; // Reference to FreeRTOS timer used for restarting wifi
int wifiCurrConAttemptsCnt = 0; // Number of Acess Point connection attempts made during current connection effort

/// @brief Define MQTT constants, classes and global variables. 
/// @note MQTT broker used for testing this was Mosquitto running on a Raspberry Pi
/// @note sends balance telemetry data to <device name><telemetry/balance> 
/// @note listens for commands on <device name><commands>
#define MQTT_BROKER_IP "192.168.2.106" // Need to make this a fixed IP address
#define MQTT_BROKER_PORT 1883 // Use 8883 for SSL
#define MQTT_USERNAME "NULL" // Not used at this time. To do: secure MQTT broker
#define MQTT_KEY "NULL" // Not used at this time. To do: secure MQTT broker
#define MQTT_IN_CMD "/commands" // Topic branch for incoming remote commands
#define MQTT_TEL_BAL "/telemetry/balance" // Topic tree for outgoing balance telemetry data
#define MQTT_METADATA "/metadata" // Topic tree for outgoing metadata about the robot
#define QOS1 1 // Quality of service level 1 ensures one time delivery
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
String cmdTopicMQTT = "NOTHING"; // Full path to incoming command topic from MQTT broker
String balTopicMQTT = "NOTHING"; // Full path to outgoing balance telemetry topic to MQTT broker 
String metTopicMQTT = "NOTHING"; // Full path to outgoing metadata topic to MQTT broker

/// @brief Define global motor control variables and structures. Also define pointers and muxing for multitasking motors via ISRs
#define motorISRus 20 // Number of microseconds between motor ISR calls 
hw_timer_t * rightMotorTimer = NULL; // Pointer to right motor ISR
hw_timer_t * leftMotorTimer = NULL; // Pointer to left motor ISR
portMUX_TYPE leftMotorTimerMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate left motor variable access between ISR and main program thread
portMUX_TYPE rightMotorTimerMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate right motor variable access between ISR and main program thread
portMUX_TYPE leftDRVMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate left DVR8825 fault error variable access between ISR and main program thread 
portMUX_TYPE rightDRVMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate right DVR8825 fault error variable access between ISR and main program thread 
typedef struct
{
  long interruptCounter; // Counter for step signals to the DRV8825 motor driver
  long riseTimeMax = 0; // Most microseconds it took for the signal rise event to happen
  long riseTimeMin = 0; // Least microseconds it took for the signal rise event to happen
  long fallTimeMax = 0; // Most microseconds it took for the signal fall event to happen
  long fallTimeMin = 0; // Least microseconds it took for the signal fall event to happen
  long delayTimeMax = 0; // Most microseconds it took for the delay time event to happen
  long delayTimeMin = 0; // Least microseconds it took for the delay time event to happen
  int motorDelay = 600; // Delay time (microseconds) used for motor speed (speed is inverse of this number)
  int tripDistance = 0; // Current distance to travel
  int tripOdometer = 0; // Current distance travelled toward trip distance
} motor; // Array for the two stepper motors that drive the robot
 static volatile motor stepperMotor[2]; // Define an array of 2 motors. 0 = right motor, 1 = left motor 

/// @brief Define global control variables.  
#define NUMBER_OF_MILLI_DIGITS 10 // Millis() uses unsigned longs (32 bit). Max value is 10 digits (4294967296ms or 49 days, 17 hours)  
#define tmrIMU 200 // Milliseconds to wait between reading data to IMU over I2C
#define tmrOLED 200 // Milliseconds to wait between sending data to OLED over I2C
#define tmrMETADATA 2000 // Milliseconds to wait between sending data to serial port
#define tmrLED 1000 / 2 // Milliseconds to wait between flashes of LED (turn on / off twice in this time)
uint32_t cntLoop = 0; // Track how many times loop() has iterated
int goIMU = 0; // Target time for next read of IMU data 
int goOLED = 0; // Target time for next OLED update
int goMETADATA = 0; // Target time for next serial port
int goLED = 0; // Target time for next toggle of LED
boolean sendMetaDataToMQTT = false; // Used to decide if metadata about the code shoud go to MQTT broker or serial port
boolean sendBalanceToMQTT = false; // Used to decide if balance telemetry data should be sent to the MQTT broker

/// @brief Define global metadata variables. Used too understand the state of the robot, its peripherals and its environment. 
int wifiConAttemptsCnt = 0; // Track the number of over all attempts made to connect to the WiFi Access Point
int mqttConAttemptsCnt = 0; // Track the number of attempts made to connect to the MQTT broker 
int dmpFifoDataMissingCnt = 0; // Track how many times the FIFO pin goes high but the buffer is empty 
int dmpFifoDataPresentCnt = 0; // Track how many times the FIFO pin goes high and there is data in the buffer 
int wifiDropCnt = 0; // Track how many times connection to the WiFi network has occurred
int mqttDropCnt = 0; // Track how many times connection to the MQTT server is lost
int unknownCmdCnt = 0; // Track how many unknown command have been recieved
int leftDRVfault = 0; // Track how many times the left DVR8825 motor driver signals a fault
int rightDRVfault = 0; // Track how many times the right DVR8825 motor driver signals a fault

/// @brief Define flags that are used to track what devices/functions are verified working after start up. Initilize false.  
boolean leftOLED_detected = false;
boolean rightOLED_detected = false;
boolean LCD_detected = false;
boolean MPU6050_detected = false;
boolean wifi_connected = false;

/** @brief Interrupt Service Routine (ISR) that runs when the DMP firmware on the MPU6050 raises its interrupt pin indicating that it
 * has data in its FIFO buffer ready to be read over I2C 
 * @note This function is not placed in IRAM 
 */
// TODO Understand the use or IRAM
void dmpDataReady() 
{
  portENTER_CRITICAL(&dmpMUX); // Prevent loop() from updating variable while we are changing it
  mpuInterrupt = true; // Flag the fact that there is data ready to be read
  portEXIT_CRITICAL(&dmpMUX); // Allow loop() access to variable again
} //dmpDataReady()

/** @brief Strips the colons off the MAC address of this device
 * @return String
 */
String formatMAC()
{
  String mac;
  AMDP_PRINTLN("<formatMAC> Removing colons from MAC address");
  mac = WiFi.macAddress(); // Get MAC address of this SOC
  mac.remove(2,1); // Remove first colon from MAC address
  mac.remove(4,1); // Remove second colon from MAC address
  mac.remove(6,1); // Remove third colon from MAC address
  mac.remove(8,1); // Remove forth colon from MAC address
  mac.remove(10,1); // Remove fifth colon from MAC address
  AMDP_PRINT("<formatMAC> Formatted MAC address without colons = ");
  AMDP_PRINTLN(mac);
  return mac;
} //formatMAC()

/**
 * @brief Control stepping of both stepper motors. 
 * @note ISR for each motor calls this routine with the motor number index.  
 * @param index Which motor the interrupt is for. 0 = right motor, 1 = left motor
 * @param mod Odometer modifier. Handle updating the trip odometer for both polarities (directions)
 */
void stepMotor(int index, uint mod) 
{
  uint8_t gpioPin[2]; // Create 2 element array to hold values of the left and right motor pins
  gpioPin[0] = gp_DRV1_STEP; // Element 0 holds right motor GPIO pin value
  gpioPin[1] = gp_DRV2_STEP; // Element 1 holds left motor GPIO pin value
  if(stepperMotor[index].interruptCounter == 1) // If this is the rising edge of the step signal
  {
    digitalWrite(gpioPin[index], HIGH);
  } //if
  if(stepperMotor[index].interruptCounter == 2) // If this is the falling edge of the step signal
  {
    digitalWrite(gpioPin[index], LOW);
  } //if
  if(stepperMotor[index].interruptCounter >= stepperMotor[index].motorDelay) // If this is the end of the delay period
  {
    portENTER_CRITICAL_ISR(&rightMotorTimerMux);
    stepperMotor[index].interruptCounter = 0;
    stepperMotor[index].tripOdometer = stepperMotor[index].tripOdometer + mod; // Update odometer here to count a single step
    portEXIT_CRITICAL_ISR(&rightMotorTimerMux);
  } //if
  else
  {
    portENTER_CRITICAL_ISR(&rightMotorTimerMux);
    stepperMotor[index].interruptCounter++;
    portEXIT_CRITICAL_ISR(&rightMotorTimerMux);  
  } //else
} //rightMotorTimerISR()

/** 
 * @brief ISR for right stepper motor that drives the robot
 * 
 */
void IRAM_ATTR rightMotorTimerISR() 
{
  int motor = 0;
  // If there is no distance to travel do nothing
  if(stepperMotor[motor].tripDistance == stepperMotor[motor].tripOdometer)
  {
    return;
  } //if  
  // Determine motor direction
  if(stepperMotor[motor].tripDistance < 0) 
  {
    digitalWrite(gp_DRV1_DIR,HIGH);
    stepMotor(motor,-1); // Manage step signalling
} //if   
  else
  {
    digitalWrite(gp_DRV1_DIR,LOW);
    stepMotor(motor,1); // Take a step
  } //else
} //rightMotorTimerISR()

/** 
 * @brief ISR for left stepper motor that drives the robot
 * 
 */
void IRAM_ATTR leftMotorTimerISR() 
{
  int motor = 1;
  // If there is no distance to travel do nothing
  if(stepperMotor[motor].tripDistance == stepperMotor[motor].tripOdometer)
  {
    return;
  } //if  
  // Determine motor direction
  if(stepperMotor[motor].tripDistance < 0) 
  {
    digitalWrite(gp_DRV2_DIR,HIGH);
    stepMotor(motor,-1); // Manage step signalling
} //if   
  else
  {
    digitalWrite(gp_DRV2_DIR,LOW);
    stepMotor(motor,1); // Take a step
  } //else
} //leftMotorTimerISR()

/**
 * @brief ISR for left DRV8825 fault condition
 * 
 */
void IRAM_ATTR leftDRV8825fault() 
{
  portENTER_CRITICAL_ISR(&leftDRVMux);
  leftDRVfault++;
  portEXIT_CRITICAL_ISR(&leftDRVMux);
} // leftDRV8825fault()

/**
 * @brief ISR for right DRV8825 fault condition
 * 
 */
void IRAM_ATTR rightDRV8825fault() 
{
  portENTER_CRITICAL_ISR(&rightDRVMux);
  rightDRVfault++;
  portEXIT_CRITICAL_ISR(&rightDRVMux);
} // rightDRV8825fault()

/** 
 * @brief This function returns a String version of the local IP address
 */
String ipToString(IPAddress ip)
{
    AMDP_PRINTLN("<ipToString> Converting IP address to String.");
    String s="";
    for (int i=0; i<4; i++)
    {
      s += i  ? "." + String(ip[i]) : String(ip[i]);
    } //for
    AMDP_PRINT("<ipToString> IP Address = ");
    AMDP_PRINTLN(s);
    return s;
} //ipToString()

/*************************************************************************************************************************************
 * @brief Connect to WiFi Access Point 
 *************************************************************************************************************************************/
void connectToWifi() 
{
  wifiConAttemptsCnt ++; // Increment the number of attempts made to connect to the Access Point 
  AMDP_PRINT("<connectToWiFi> Attempt #");
  AMDP_PRINT(wifiConAttemptsCnt);
  AMDP_PRINTLN(" to connect to a WiFi Access Point");
  WiFi.begin(mySSID, myPassword);
} //connectToWifi()

/*************************************************************************************************************************************
 * @brief Connect to MQTT broker
 * @note MQTT Spec: https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718063
 * @note Reference for MQTT comments in this code: https://www.hivemq.com/mqtt-essentials/
 *************************************************************************************************************************************/
void connectToMqtt() 
{
  AMDP_PRINTLN("<connectToMqtt> Connecting to MQTT...");
  mqttClient.connect();
  mqttConAttemptsCnt ++; // Increment the number of attempts made to connect to the MQTT broker 
} //connectToMqtt()

/*************************************************************************************************************************************
 * @brief Handle all WiFi events
 * @param event WiFi event that caused this function to be called
 * # WiFi Event Handling
 * All Wifi events are processed by the WiFiEvent method. A list of the events appears in the table below.
 * 
 * ## Table of WiFi Events
* | Return Code | Constant Directive | Description                                                       |  
 * |:-----------:|:--------------------------------------------------------------------------------------|
 * | 0  | SYSTEM_EVENT_WIFI_READY | WiFi ready |
 * | 1  | SYSTEM_EVENT_SCAN_DONE | finish scanning AP |
 * | 2  | SYSTEM_EVENT_STA_START | station start |
 * | 3  | SYSTEM_EVENT_STA_STOP | station stop |
 * | 4  | SYSTEM_EVENT_STA_CONNECTED | station connected to AP |
 * | 5  | SYSTEM_EVENT_STA_DISCONNECTED | station disconnected from AP |
 * | 6  | SYSTEM_EVENT_STA_AUTHMODE_CHANGE | the auth mode of AP connected by ESP32 station changed |
 * | 7  | SYSTEM_EVENT_STA_GOT_IP | station got IP from connected AP |
 * | 8  | SYSTEM_EVENT_STA_LOST_IP | station lost IP and the IP is reset to 0 |
 * | 9  | SYSTEM_EVENT_STA_WPS_ER_SUCCESS | station wps succeeds in enrollee mode |
 * | 10 | SYSTEM_EVENT_STA_WPS_ER_FAILED | station wps fails in enrollee mode |
 * | 11 | SYSTEM_EVENT_STA_WPS_ER_TIMEOUT | station wps timeout in enrollee mode |
 * | 12 | SYSTEM_EVENT_STA_WPS_ER_PIN | station wps pin code in enrollee mode |
 * | 13 | SYSTEM_EVENT_AP_START | soft-AP start |
 * | 14 | SYSTEM_EVENT_AP_STOP | soft-AP stop |
 * | 15 | SYSTEM_EVENT_AP_STACONNECTED | a station connected to ESP32 soft-AP |
 * | 16 | SYSTEM_EVENT_AP_STADISCONNECTED | a station disconnected from ESP32 soft-AP |
 * | 17 | SYSTEM_EVENT_AP_STAIPASSIGNED | soft-AP assign an IP to a connected station |
 * | 18 | SYSTEM_EVENT_AP_PROBEREQRECVED | Receive probe request packet in soft-AP interface |
 * | 19 | SYSTEM_EVENT_GOT_IP6 | station or ap or ethernet interface v6IP addr is preferred |
 * | 20 | SYSTEM_EVENT_ETH_START | ethernet start |
 * | 21 | SYSTEM_EVENT_ETH_STOP | ethernet stop |
 * | 22 | SYSTEM_EVENT_ETH_CONNECTED | ethernet phy link up |
 * | 23 | SYSTEM_EVENT_ETH_DISCONNECTED | ethernet phy link down |
 * | 24 | SYSTEM_EVENT_ETH_GOT_IP | ethernet got IP from connected AP |
 * | 25 | SYSTEM_EVENT_MAX | |
 *************************************************************************************************************************************/
void WiFiEvent(WiFiEvent_t event) 
{
  String tmpHostNameVar; // Hold WiFi host name created in this function
  AMDP_PRINT("<WiFiEvent> event:");  
  AMDP_PRINTLN(event);
  switch(event) 
  {
    case SYSTEM_EVENT_STA_CONNECTED:
    {
      AMDP_PRINTLN("<WiFiEvent> Event 4 = Got connected to Access Point");
      break;        
    } //case
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
      AMDP_PRINTLN("<WiFiEvent> Lost WiFi connection");
      int blockTime  = 10; // https://www.freertos.org/FreeRTOS-timers-xTimerStart.html
      xTimerStop(mqttReconnectTimer, blockTime); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi. Disconnect triggers new connect atempt
      xTimerStart(wifiReconnectTimer, blockTime); // Activate wifi timer (which only runs 1 time)
      wifi_connected = false;
      wifiDropCnt ++; // Increment the number of network drops that have occured
      break;      
    } //case
    case SYSTEM_EVENT_STA_GOT_IP:
    {
      AMDP_PRINT("<WiFiEvent> Event 7 = Got IP address. That address is: ");
      AMDP_PRINTLN(WiFi.localIP());
      myIPAddress = ipToString(WiFi.localIP());
      myAccessPoint = WiFi.SSID();
      tmpHostNameVar = myHostNameSuffix + myMACaddress;
      WiFi.setHostname((char*)tmpHostNameVar.c_str());
      myHostName = WiFi.getHostname();
      Serial.print("<WiFiEvent> Network connecion attempt #");
      Serial.print(wifiConAttemptsCnt);
      Serial.print(" SUCCESSFUL after this many tries: ");
      Serial.println(wifiCurrConAttemptsCnt);
      Serial.println("<WiFiEvent> Network information is as follows..."); 
      Serial.print("<WiFiEvent> - Access Point Robot is connected to = ");
      Serial.println(myAccessPoint);
      Serial.print("<WiFiEvent> - Robot Network Host Name = ");
      Serial.println(myHostName);
      Serial.print("<WiFiEvent> - Robot IP Address = ");
      Serial.println(myIPAddress);
      Serial.print("<WiFiEvent> - Robot MAC Address = ");
      Serial.println(myMACaddress);    
      wifi_connected = true;
      AMDP_PRINTLN("<WiFiEvent> Use MAC address to create MQTT topic trees...");
      cmdTopicMQTT = myHostName + MQTT_IN_CMD; // Define variable with the full name of the incoming command topic
      balTopicMQTT = myHostName + MQTT_TEL_BAL; // Define variabe with the full name of the outgoing balance telemetry topic
      metTopicMQTT = myHostName + MQTT_METADATA; // Define variable with full name of the outgoiong metadata topic
      AMDP_PRINT("<WiFiEvent> cmdTopicMQTT = ");
      AMDP_PRINTLN(cmdTopicMQTT);
      AMDP_PRINT("<WiFiEvent> balTopicMQTT = ");
      AMDP_PRINTLN(balTopicMQTT);
      AMDP_PRINT("<WiFiEvent> metTopicMQTT = ");
      AMDP_PRINTLN(metTopicMQTT);
      connectToMqtt();
      break;
    } //case
    default:
    {
      AMDP_PRINT("<WiFiEvent> Detected unmanaged WiFi event ");
      AMDP_PRINTLN(event);
    } //default
  } //switch
} //WiFiEvent(WiFiEvent_t event)

/*************************************************************************************************************************************
 * @brief Handle CONNACK from the MQTT broker
 * @param sessionPresent persitent session available flag contained in CONNACK message from MQTT broker.
 * # CONNACK Message
 * The CONNACK message contains two data entries:
 * 1. The session present flag
 * 2. A connect acknowledge flag
 * 
 * ## Session Present Flag
 * The session present flag tells the client whether the broker already has a persistent session available from previous interactions 
 * with the client. When a client connects with Clean Session set to true, the session present flag is always false because there is no 
 * session available. If a client connects with Clean Session set to false, there are two possibilities: If session information is 
 * available for the client Id. and the broker has stored session information, the session present flag is true. Otherwise, if the 
 * broker does not have any session information for the client ID, the session present flag is false. This flag was added in MQTT 3.1.1 
 * to help clients determine whether they need to subscribe to topics or if the topics are still stored in a persistent session.
 *
 * ## Connect Acknowledge Flag
 * The second flag in the CONNACK message is the connect acknowledge flag. This flag contains a return code that tells the client 
 * whether the connection attempt was successful. Here are the return codes at a glance:
 *
 * | Return Code | Return Code Response |  
 * |:-----------:|:------------------------------------------------------------------------------------------------------------------|
 * |  0  | Connection accepted |
 * |  1  | Connection refused, unacceptable protocol version |
 * |  2  | Connection refused, identifier rejected |
 * |  3  | Connection refused, server unavailable |
 * |  4  | Connection refused, bad user name or password |
 * |  5  | Connection refused, not authorized |
 *************************************************************************************************************************************/
void onMqttConnect(bool sessionPresent) 
{
  AMDP_PRINTLN("<onMqttConnect> Connected to MQTT");
  AMDP_PRINT("<onMqttConnect> Session present: ");
  AMDP_PRINTLN(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe((char*)cmdTopicMQTT.c_str(), QOS1); // QOS can be 0,1 or 2. We are using 1
  Serial.print("<onMqttConnect> Subscribing to ");
  Serial.print(cmdTopicMQTT);
  Serial.print(" at a QOS of 1 with a packetId of ");
  Serial.println(packetIdSub);
} //onMqttConnect()

/*************************************************************************************************************************************
 * @brief Handle disconnecting from an MQTT broker
 * @param reason Reason for disconnect
 *************************************************************************************************************************************/
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) 
{
  AMDP_PRINTLN("<onMqttDisconnect> Disconnected from MQTT");
  mqttDropCnt ++; // Increment the counter for the number of MQTT connection drops
  mqttConAttemptsCnt++;
  if (WiFi.isConnected()) 
  {
    xTimerStart(mqttReconnectTimer, 0); // Activate mqtt timer (which only runs 1 time)
  } //if
  mqttConAttemptsCnt = 0; // Reset the number of attempts made to connect to the MQTT broker 
} //onMqttDisconnect()

/*************************************************************************************************************************************
 * @brief Handle SUBACK from MQTT broker
 * @param packetId Unique identifier of the message
 * @param qos SUBACK return code
 * # Suback Message
 * To confirm each subscription, the broker sends a SUBACK acknowledgement message to the client. This message contains the packet 
 * identifier of the original Subscribe message (to clearly identify the message) and a list of return codes.
 * ## Packet Identifier 
 * The packet identifier is a unique identifier used to identify a message. It is the same as in the SUBSCRIBE message.
 *
 * ##Return Code 
 * The broker sends one return code for each topic/QoS-pair that it receives in the SUBSCRIBE message. For example, if the SUBSCRIBE 
 * message has five subscriptions, the SUBACK message contains five return codes. The return code acknowledges each topic and shows 
 * the QoS level that is granted by the broker. If the broker refuses a subscription, the SUBACK message conains a failure return code 
 * for that specific topic. For example, if the client has insufficient permission to subscribe to the topic or the topic is malformed.
 *
 * # Table of SUBACK return codes
 * | Return Code | Return Code Response |  
 * |:-----------:|:------------------------------------------------------------------------------------------------------------------|
 * |  0  |  Success - Maximum QoS 0 |
 * |  1  |  Success - Maximum QoS 1 |
 * |  2  |  Success - Maximum QoS 2 |
 * | 128 |  Failure |
 *************************************************************************************************************************************/
void onMqttSubscribe(uint16_t packetId, uint8_t qos) 
{
  AMDP_PRINTLN("<onMqttSubscribe> Subscribe acknowledged by broker.");
  AMDP_PRINT("<onMqttSubscribe>  PacketId: ");
  AMDP_PRINTLN(packetId);
  AMDP_PRINT("<onMqttSubscribe>  QOS: ");
  AMDP_PRINTLN(qos);
} //onMqttSubscribe

/*************************************************************************************************************************************
 * @brief Handle UNSUBACK message from MQTT broker
 * @param packetId Unique identifier of the message. This is the same packet identifier that is in the UNSUBSCRIBE message.
 * # UNSUBACK Message
 * To confirm the unsubscribe, the broker sends an UNSUBACK acknowledgement message to the client. This message contains only the 
 * packet identifier of the original UNSUBSCRIBE message (to clearly identify the message). After receiving the UNSUBACK from the 
 * broker, the client can assume that the subscriptions in the UNSUBSCRIBE message are deleted.
 *************************************************************************************************************************************/
void onMqttUnsubscribe(uint16_t packetId) 
{
  AMDP_PRINTLN("Unsubscribe acknowledged.");
  AMDP_PRINT("  packetId: ");
  AMDP_PRINTLN(packetId);
} //onMqttUnsubscribe()

/*************************************************************************************************************************************
 * @brief Handle incoming messages from MQTT broker for topics subscribed to
 * @param topic Which topic this message if about
 * @param payload The content of the message sent from the MQTT broker
 * @param properties A structure of flags with details from the message's packet header 
 * @param len Payload length. If unset or set to 0, the payload will be considered as a string and use strlen(payload) to calculte
 * @param index The message ID. If unset or set to 0, the message ID will be automtaically assigned
 * @param total Seems to be the same as len 
 * 
 * # Property Flags
 * The MQTT message has a header containing the following flags.
 * ## QOS Flag
 * Two bits that show the QOS level of the message. Valid values are 1, 2 and 3
 * ## Retain Flag
 * Boolean value showing whether the message is saved by the broker as the last known good value for a specified topic. When a new 
 * client subscribes to a topic, they receive the last message that is retained on that topic. 
 * ## DUP flag 
 * Boolean value indicates that the message is a duplicate and was resent because the intended recipient (client or broker) did not 
 * acknowledge the original message. This is only relevant for QoS greater than 0.
 *************************************************************************************************************************************/
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) 
{
  AMDP_PRINT("<onMqttMessage> Publish received.");
  AMDP_PRINT("<onMqttMessage>  topic: ");
  AMDP_PRINTLN(topic);
  AMDP_PRINT("<onMqttMessage>  qos: ");
  AMDP_PRINTLN(properties.qos);
  AMDP_PRINT("<onMqttMessage>  dup: ");
  AMDP_PRINTLN(properties.dup);
  AMDP_PRINT("<onMqttMessage>  retain: ");
  AMDP_PRINTLN(properties.retain);
  AMDP_PRINT("<onMqttMessage>  len: ");
  AMDP_PRINTLN(len);
  AMDP_PRINT("<onMqttMessage>  index: ");
  AMDP_PRINTLN(index);
  AMDP_PRINT("<onMqttMessage>  total: ");
  AMDP_PRINTLN(total);
  AMDP_PRINT("<onMqttMessage>  payload: ");
  AMDP_PRINTLN(payload);

  /// @brief Look at the incoming command and decide what to do with it 
  /// # Commands
  /// When WiFi is available and  there is an MQTT broker availabe, TWIPe is always subscibed to the topic {robot name}/commands. All
  /// incoming messages from that topic are checked against a known list of commands and get processed accordingly. Commands that are
  /// not recognized get logged and ignored.
  ///  ## Table of Known Commands
  /// | Command                | Description                                                                                            |
  /// |:-----------------------|:-------------------------------------------------------------------------------------------------------|
  /// | balTelON               | Causes balance telemetry data to be published to the MQTT broker topic {robot name}/telemetry/balance  |  
  /// | balTelOFF              | Causes balance telemetry data to stop being published to the MQTT broker |  
  /// | metaDataON             | Causes metadata to be published to the MQTT broker topic {robot name}/metadata |  
  /// | metaDataOFF            | Causes metadata to stop being published to the MQTT broker |  
  /// | motor,num,steps,delay  | Manual motor control. Use +/- step values for direction |
  String tmp = String(payload).substring(0,len);
  AMDP_PRINT("<onMqttMessage> Message to process = ");
  AMDP_PRINTLN(tmp);
  if(tmp.substring(0,5) == "motor")
  {
    int Index1 = tmp.indexOf(','); // Command motor
    int Index2 = tmp.indexOf(',', Index1+1); // Motor number (1 or 2)
    int Index3 = tmp.indexOf(',', Index2+1); // Motor speed
    int Index4 = tmp.indexOf(',', Index3+1); // Motor direction
    String mNum = tmp.substring(Index1+1, Index2);
    String mSteps = tmp.substring(Index2+1, Index3);
    String mDelay = tmp.substring(Index3+1, Index4);  
    int motNum = atoi(mNum.c_str());
    uint motStep = atoi(mSteps.c_str());
    int motDelay = atoi(mDelay.c_str());
    if(motNum == 0)
    {
      portENTER_CRITICAL(&rightMotorTimerMux); // Prevent loop() from updating variable while we are changing it
      stepperMotor[motNum].tripDistance = motStep;
      stepperMotor[motNum].tripOdometer = 0;
      stepperMotor[motNum].motorDelay = motDelay; 
      portEXIT_CRITICAL(&rightMotorTimerMux); // Allow loop() access to variable again
    } //if
    else
    {
      portENTER_CRITICAL(&leftMotorTimerMux); // Prevent loop() from updating variable while we are changing it
      stepperMotor[motNum].tripDistance = motStep;
      stepperMotor[motNum].tripOdometer = 0;
      stepperMotor[motNum].motorDelay = motDelay; 
      portEXIT_CRITICAL(&leftMotorTimerMux); // Allow loop() access to variable again
    } //else
    AMDP_PRINTLN("<onMqttMessage> Manual motor control command");    
    AMDP_PRINT("<onMqttMessage> Motor number = ");    
    AMDP_PRINTLN(motNum);    
    AMDP_PRINT("<onMqttMessage> Steps = ");    
    AMDP_PRINTLN(stepperMotor[motNum].tripDistance);    
    AMDP_PRINT("<onMqttMessage> Delay (us) = ");    
    AMDP_PRINTLN(stepperMotor[motNum].motorDelay);    
  } //if
  else if(tmp == "balTelON")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of telemetry data to MQTT broker now ON");
    sendBalanceToMQTT = true;
  } //if
  else if(tmp == "balTelOFF")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of telemetry data to MQTT broker now OFF");
    sendBalanceToMQTT = false;
  } //if
  else if(tmp == "metaDataON")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of metadata to MQTT broker now ON");
    sendMetaDataToMQTT = true;
  } //if
  else if(tmp == "metaDataOFF")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of metadata to MQTT broker now OFF");
    sendMetaDataToMQTT = false;
  } //if
  else
  {
    AMDP_PRINTLN("<onMqttMessage> Unknown command. Doing nothing");
    unknownCmdCnt++; // Increment the counter that tracks how many unknown commands have been recieved
  } //else
} //onMqttMessage()

/*************************************************************************************************************************************
 * @brief Handle the reciept of a PUBACK message message from MQTT broker
 * @param packetId Unique identifier of the message.
 *************************************************************************************************************************************/
void onMqttPublish(uint16_t packetId) 
{
  AMDP_PRINTLN("Publish acknowledged.");
  AMDP_PRINT("  packetId: ");
  AMDP_PRINTLN(packetId);
} //onMqttPublish()

/*************************************************************************************************************************************
 This function returns a String version of the local IP address.
 *************************************************************************************************************************************/
void connectToNetwork() 
{
  int maxConnectionAttempts = 20; // Maximum number of Access Point connection attemts 
  wifiCurrConAttemptsCnt = 0; // Number of Access Point connection attempts made during current connection/reconnection effort
  String tmpHostNameVar; // Hold WiFi host name created in this function
  AMDP_PRINT("<connectToNetwork> Try connecting to Access Point ");
  AMDP_PRINTLN(mySSID);
  WiFi.onEvent(WiFiEvent); // Create a WiFi event handler
  connectToWifi();
  while ((WiFi.status() != WL_CONNECTED) && (maxConnectionAttempts > 0)) 
  {
    delay(1000);
    AMDP_PRINT("<connectToNetwork> Establishing connection to WiFi. Connect attempt count down = ");
    AMDP_PRINTLN(maxConnectionAttempts);
    maxConnectionAttempts--;
    wifiCurrConAttemptsCnt++;
  } //while  
  if(WiFi.status() != WL_CONNECTED)
  {
    AMDP_PRINTLN("<connectToNetwork> Connection to network FAILED");
  } //if
} //connectToNetwork() 

/*************************************************************************************************************************************
 * @brief This function translates the type of encryption that an Access Point (AP) advertises (an an ENUM) 
 * and returns a more human readable description of what that encryption method is.
 *************************************************************************************************************************************/
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

/*************************************************************************************************************************************
 * @brief This function scans the WiFi spectrum looking for Access Points (AP). It selects the AP with the 
 * strongest signal which is included in the known network list.
 *************************************************************************************************************************************/
void scanNetworks() 
{
  int numberOfNetworks = WiFi.scanNetworks(); // Used to track how many APs are detected by the scan
  int StrongestSignal = -127; // Used to find the strongest signal. Set as low as possible to start
  int SSIDIndex = 0; // Contains the SSID index number from the known list of APs
  bool APknown; // Flag to indicate if the current AP appears in the known AP list
  AMDP_PRINTLN("<scanNetworks> Scanning for WiFi Access Points.");
  AMDP_PRINT("<scanNetworks> Number of networks found: ");
  AMDP_PRINTLN(numberOfNetworks);
  
  // Loop through all detected APs
  for (int i = 0; i < numberOfNetworks; i++) 
  {
    APknown = false;
    AMDP_PRINT("<scanNetworks> Network name: ");
    AMDP_PRINTLN(WiFi.SSID(i));
    AMDP_PRINT("<scanNetworks> Signal strength: ");
    AMDP_PRINTLN(WiFi.RSSI(i));
    AMDP_PRINT("<scanNetworks> MAC address: ");
    AMDP_PRINTLN(WiFi.BSSIDstr(i));
    AMDP_PRINT("<scanNetworks> Encryption type: ");
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
        AMDP_PRINTLN("<scanNetworks> This is a known network");
      } //if
    } //for

    // If the current AP is known and has a stronger signal than the others that have been checked 
    // then store it in the variables that will be used to connect to the AP later
    if((APknown == true) && (WiFi.SSID(i).toInt() > StrongestSignal))
    {
      mySSID = SSID[SSIDIndex].c_str();
      myPassword = Password[SSIDIndex].c_str();
      StrongestSignal = WiFi.SSID(i).toInt();
      AMDP_PRINTLN("<scanNetworks> This is the strongest signal so far");
    } //if
    AMDP_PRINTLN("<scanNetworks> -----------------------");
  } //for

  AMDP_PRINT("<scanNetworks> Best SSID candidate = ");
  AMDP_PRINTLN(mySSID);
} //scanNetworks()

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
 * @brief Publish a message to the specified MQTT broker topic tree
 * @param topic The topic tree to publish the messge to
 * @param msg The message to send
 * # MQTT Published Messages
 * TWIPe issues a number of messages to the MQTT broker. Each message goes to it's own topic tree. These messages and their 
 * respective topic trees are outlined in the table below. Note hat the robot name is made up of the prefix twipe followed by the 
 * MAC address of the ESP32. This acts as a unique identifier of the robot and is used to ensure that each robot has its own 
 * unique data tree structure on the MQTT broker. THis design allows for multipple twipe robots to share the same MQTT broker.
 * ## Table of Published Messages 
 * | Topic                  | Topic Tree                       | Details                                                             |
 * |:-----------------------|:---------------------------------|:--------------------------------------------------------------------|
 * | Balance telemetry      | {robot name}/telemetry/balance   | Angle of IMU orientation in degrees                                 |
 * | Robot Metadata         | {robot name}/metadata            | See metadata table for a full list of the data points being tracked |
 *************************************************************************************************************************************/
void publishMQTT(String topic, String msg) 
{
  char tmp[NUMBER_OF_MILLI_DIGITS];
  itoa(millis(), tmp, NUMBER_OF_MILLI_DIGITS);
  String message = String(tmp) + "," + msg;
  if(topic == "balance")
  {
    uint16_t packetIdPub1 = mqttClient.publish((char*)balTopicMQTT.c_str(), QOS1, false, (char*)message.c_str()); // QOS 0-2, retain t/f  
    AMDP_PRINT("<publishMQTT> PacketID for publish to balance topic is ");
    AMDP_PRINTLN(packetIdPub1);
  } //if
  else if(topic == "metadata")
  {
    uint16_t packetIdPub1 = mqttClient.publish((char*)metTopicMQTT.c_str(), QOS1, false, (char*)message.c_str()); // QOS 0-2, retain t/f  
    AMDP_PRINT("<publishMQTT> PacketID for publish to metadata topic is ");
    AMDP_PRINTLN(packetIdPub1);
  } //else if
  else
  {
    AMDP_PRINT("<publishMQTT> ERROR. Unknown MQTT topic tree - ");
    AMDP_PRINTLN(topic);
  } //else
} //publishMQTT()

/*************************************************************************************************************************************
 * @brief Calculate angle and reformat result of calculation from float to String to pass along to publishMQTT() 
 * @param angle Angle of robot lean in eulers. To convert to degrees use this formula: degrees = angle * 180 / PI
 * @note We are working with Yaw/Pitch/Roll data (and only using pitch). Other options include euler, quaternion, raw acceleration, 
 * raw gyro, linear acceleration and gravity. See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
 *************************************************************************************************************************************/
void formatBalanceData(float angle)
{      
  publishMQTT("balance", String(angle * 180 / PI));
} //formatBalanceData()

/*************************************************************************************************************************************
 * @brief Send updated metadata about the running of the code.
 * # Metadata
 * There are a number of data points that the TWIPe code tracks in order to assess how the robot's logic is performing. These data 
 * points can be used to pinpoint the cause of perfomance issues as well as play a key role in debugging. Metadata is always being 
 * issued from the updateMetaData() function. When the metaDataON command is recieved all metadata is directed to the MQTT 
 * broker. When the command metaDataOFF is recieved all metadata is directed to the serial port. NOte that all the the items in the 
 * table below are found in the payload of the MQTT message. The payload starts with a time stamp in millis() and then is followed 
 * by each of the items in the table below. Each item is seperated by a comma. 
 * ## Table of metadata tracked 
 * | Item                     | Details                                                                                               |
 * |:-------------------------|:------------------------------------------------------------------------------------------------------|
 * | WiFi connection          | Number of times a wifi connection had to be established |
 * | WiFi drop                | Number of times the wifi connection dropped |
 * | MQTT connection          | Number of times an MQTT connection had to be established |
 * | MQTT drop                | Number of times an MQTT connection dropped     
 * | MPU6050 DMP read success | Successful attempts to read from the MPU6050 DMP FIFO buffer  |
 * | MPU6050 DMP read fails   | Failed attempts to read from the MPU6050 DMP FIFO buffer  |
 * | Unknown command          | Number of unrecognized commands have been recieved. |
 * | Left DRV8825 fault       | Number of fault signals sent by the left DVR8825 stepper motor driver |
 * | Right DRV8825 fault      | Number of fault signals sent by the right DVR8825 stepper motor driver |
 *************************************************************************************************************************************/
void updateMetaData()
{
  String tmp = String(wifiConAttemptsCnt);
  tmp += "," + String(wifiDropCnt);
  tmp += "," + String(mqttConAttemptsCnt);
  tmp += "," + String(mqttDropCnt);
  tmp += "," + String(dmpFifoDataPresentCnt);
  tmp += "," + String(dmpFifoDataMissingCnt); 
  tmp += "," + String(unknownCmdCnt); 
  tmp += "," + String(leftDRVfault); 
  tmp += "," + String(rightDRVfault); 
  
  if(sendMetaDataToMQTT && wifi_connected) /// If configured to send metadata to MQTT and there is a wifi connection send to MQTT broker 
  {
    publishMQTT("metadata", tmp);
  } //if
  else // /Otherwise send to serial port
  {
    AMDP_PRINT("<updateMetaData> ");
    AMDP_PRINTLN(tmp);
  } //else
  goMETADATA = millis() + tmrMETADATA; // Reset SERIAL update target time
} // updateMetaData()

/*************************************************************************************************************************************
 * @brief Update OLED dipsplay 
 * @param angle Angle of robot lean in eulers. To convert to degrees use this formula: degrees = angle * 180 / PI
 * @note We are working with Yaw/Pitch/Roll data (and only using pitch). Other options include euler, quaternion, raw acceleration, 
 * raw gyro, linear acceleration and gravity. See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
 *************************************************************************************************************************************/
void updateOLED(float angle)
{
  rightOLED.clear();        
  rightOLED.drawString(64, 20, String(angle * 180 / PI));
  rightOLED.display();  
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
 * @brief Function called when a message appears on the command topic subsription 
 * @param topic 
 * @param message 
 *************************************************************************************************************************************/
void subscribed_callback(char *data, uint16_t len)
{
  // Print out topic name and message
  AMDP_PRINT("<subscribed_callback> Got this command: ");
  AMDP_PRINTLN(data);
} //subscribed_callback

/*************************************************************************************************************************************
 * @brief Set up communication with an MQTT broker
 * Refer to: https://learn.adafruit.com/introducing-the-adafruit-wiced-feather-wifi/adafruitmqtt
 *************************************************************************************************************************************/
void setupMQTT()
{
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
} //setupMQTT()

/*************************************************************************************************************************************
 * @brief Set up the LED that is flashed by loop()
 *************************************************************************************************************************************/
void setupLED()
{
  AMDP_PRINTLN("<setupLED> Enable LED pin");
  pinMode(gp_SWC_LED, OUTPUT); // configure LED for output
} //setupLED()

/*************************************************************************************************************************************
 * @brief Set up the OLED 
 *************************************************************************************************************************************/
void setupOLED()
{
  AMDP_PRINTLN("<setupOLED> Initialize OLED");
  rightOLED.init();
  rightOLED.setFont(ArialMT_Plain_24);
  rightOLED.setTextAlignment(TEXT_ALIGN_CENTER);
  rightOLED.drawString(64, 20, "My Demo"); //64,22  
  rightOLED.display();
  AMDP_PRINTLN("<setupOLED> Initialization of OLED complete");
} //setupOLED()

/*************************************************************************************************************************************
 * @brief Set up the MPU6050 using DMP firmware and interrupts
 *************************************************************************************************************************************/
void setupIMU()
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief initialize device
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  AMDP_PRINTLN("<setupIMU> Initializing MPU6050...");
  mpu.initialize();
  pinMode(gp_IMU_INT, INPUT);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief verify connection
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  AMDP_PRINTLN("<setupIMU> Testing MPU6050 connection...");
  bool tmp = mpu.testConnection();
  if(tmp == true)
  {
    AMDP_PRINTLN("<setupIMU> MPU6050 connection successful");
  } //if
  else
  {
    AMDP_PRINTLN("<setupIMU> MPU6050 connection failed. Halting boot up");
    while(1);
  } //else
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// @brief load and configure the DMP
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  AMDP_PRINTLN(F("<setupIMU> Initializing DMP..."));
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
    AMDP_PRINTLN();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    AMDP_PRINTLN("<setupIMU> Enabling DMP...");
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    AMDP_PRINT("<setupIMU> Enabling DMP FIFO data ready GPIO pin ");
    AMDP_PRINTLN(digitalPinToInterrupt(gp_IMU_INT));
    AMDP_PRINTLN("<setupIMU> Attaching inetrrupt pin to dmpDataReady function");
    attachInterrupt(digitalPinToInterrupt(gp_IMU_INT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    AMDP_PRINT("<setupIMU> MPU initialization status = ");
    AMDP_PRINTLN(mpuIntStatus);
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    AMDP_PRINT("<setupIMU> packetSize = ");
    AMDP_PRINTLN(packetSize);
    // Initial read of DMP FIFO  
  } //if
  else // If initialization failed
  {
    Serial.print("<setupIMU> DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.print(") = ");
    if(devStatus == 1)
    {
      Serial.println("initial memory load failed");
    } //if
    else if(devStatus == 2)
    {
      Serial.println("DMP configuration updates failed");
    } //if
    else
    {
      Serial.println("cause of failure unknown");
    } //if
    Serial.println("<setupIMU> Boot sequence halted");
    while(1); // loop forever thus halting boot up
  } //else
} //setupIMU()

/**
 * @brief Set up robot specific configuration based on the ESP32 MAC address
 */
void cfgByMAC()
{
  myMACaddress = formatMAC();
  if(myMACaddress == "BCDDC2F7D6D5") // This is Andrew's bot
  {
    AMDP_PRINTLN("<cfgByMAC> Setting up MAC BCDDC2F7D6D5 configuration - Andrew");
    XGyroOffset = 135;
    YGyroOffset = -9;
    ZGyroOffset = -85;
    XAccelOffset = -3396;
    YAccelOffset = 830;
    ZAccelOffset = 1890;  
    wheelDiameter = 3.75; 
    stepsPerRevolution = 200;
    heightCOM = 5;
  } //if
  else if(myMACaddress == "B4E62D9EA8F9") // This is Doug's bot
  {
    AMDP_PRINTLN("<cfgByMAC> Setting up MAC BCDDC2F7D6D5 configuration - Doug");
    XGyroOffset = 60;
    YGyroOffset = -10;
    ZGyroOffset = -72;
    XAccelOffset = -2070;
    YAccelOffset = -70;
    ZAccelOffset = 1641;      
    wheelDiameter = 3.75; 
    stepsPerRevolution = 200;
    heightCOM = 5;
  } //else if
  else
  {
    Serial.println("<cfgByMAC> MAC not recognized. Setting up generic configuration");
    XGyroOffset = 135;
    YGyroOffset = -9;
    ZGyroOffset = -85;
    XAccelOffset = -3396;
    YAccelOffset = 830;
    ZAccelOffset = 1890;      
    wheelDiameter = 3.75; 
    stepsPerRevolution = 200;
    heightCOM = 5;
  } //else
  wheelCircumference = wheelDiameter * PI; 
  distancePerStep = wheelCircumference / stepsPerRevolution;
  Serial.print("<cfgByMAC> Distance per step = ");
  Serial.println(distancePerStep);
} //cfgByMAC()

/**
 * @brief Flash AMBER LED on fron panel button
 */
void updateLED()
{
  blinkState = !blinkState;
  digitalWrite(gp_SWC_LED, blinkState);
  goLED = millis() + tmrLED; // Reset LED flashing counter
} // updateLED()

/**
 * @brief Retrieve DMP FIFO data
 */
// TODO learn about the three different dmpGet commands used here. Do we need them all? What do they do? an we call only 1? 
void readIMU()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Check to see if there is any data in the DMP FIFO buffer. 
  {  
    mpu.dmpGetQuaternion(&q, fifoBuffer); // Get the latest packet of Quaternion data
    mpu.dmpGetGravity(&gravity, &q); // Get the latest packet of gravity data 
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get the latest packet of Euler angles 
    dmpFifoDataPresentCnt++; // Track how many times the FIFO pin goes high and the buffer has data in it
  } //if
  else // If DMP pin goes high but there is no data in the FIFO buffer then something weird happend
  {
    dmpFifoDataMissingCnt++; // Track how many times the FIFO pin goes high but the buffer is empty  
  } //else
  if(sendBalanceToMQTT) 
  {
    formatBalanceData(ypr[2]);
  }//if
  goIMU = millis() + tmrIMU; // Reset IMU update counter  
} // readIMU()

/**    
 * @brief Create FreeRTOS timers that run callback functions in their own seperate FreeRTOS threads. 
 * @note For details abut FreeRTOS timers see https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html
 * @note Timers are created but are not started at this point. xTimerStart is used later to start them.
 */
void setupFreeRTOStimers()
{
  int const wifiTimerPeriod = 2000; // Time in milliseconds between wifi timer events 
  int const mqttTimerPeriod = 2000; // Time in milliseconds between mqtt timer events 
  mqttReconnectTimer = xTimerCreate("mqttTimer", // Human readable name assigned to timer
                                    pdMS_TO_TICKS(mqttTimerPeriod), // set timer period. pdMS_TO_TICKS() converts milliseconds to ticks
                                    pdFALSE, // Set reload to FALSE so this timer becomes dormant after one run
                                    (void*)0, // Timer ID. Not used in our callback function 
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt)); // Function the timer calls when it expires 
  wifiReconnectTimer = xTimerCreate("wifiTimer", // Human readable name assigned to timer
                                    pdMS_TO_TICKS(wifiTimerPeriod), // set timer period. pdMS_TO_TICKS() converts milliseconds to ticks
                                    pdFALSE, // Set reload to FALSE so this timer becomes dormant after one run
                                    (void*)0, // Timer ID. Not used in our callback function 
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi)); // Function the timer calls when it expires 
  if (mqttReconnectTimer == NULL) // Check result of xTimerCreate for mqtt timer
  {
    Serial.println("<setupFreeRTOStimers> Error. mqttTimer thread was not created");
  } //if
  if (wifiReconnectTimer == NULL)
  {
    Serial.println("<setupFreeRTOStimers> Error. wifiTimer thread was not created");
  } //if
} // setupFreeRTOStimers()

/** @brief Configure GPIO pins for stepper motors
 * 
 */
void setupDriverMotors()
{
  // Set up GPIO pins for the robot's right motor
  AMDP_PRINTLN("<setupDriverMotors> Initialize GPIO pins for right motor");  
  pinMode(gp_DRV1_DIR, OUTPUT); // Set left direction pin as output
  pinMode(gp_DRV1_STEP, OUTPUT); // Set left step pin as output
  pinMode(gp_DRV1_ENA, OUTPUT); // Set left enable pin as output
  pinMode(gp_DRV1_FAULT, INPUT); // Set left driver fault pin as input
  digitalWrite(gp_DRV1_DIR, LOW); // Set left motor direction as forward  
  digitalWrite(gp_DRV1_ENA, LOW); // Enable right motor  
  // Set up GPIO pins for the robot's left motor
  AMDP_PRINTLN("<setupDriverMotors> Initialize GPIO pins for left motor");  
  pinMode(gp_DRV2_DIR, OUTPUT); // Set right direction pin as output
  pinMode(gp_DRV2_STEP, OUTPUT); // Set right step pin as output
  pinMode(gp_DRV2_ENA, OUTPUT); // Set right enable pin as output
  pinMode(gp_DRV2_FAULT, INPUT); // Set right driver fault pin as input
  digitalWrite(gp_DRV2_DIR, LOW); // Set right motor direction as forward  
  digitalWrite(gp_DRV2_ENA, LOW); // Enable left motor 
  // Set up right motor driver ISR
  AMDP_PRINTLN("<setupDriverMotors> Configure timer0 to control the right motor");  
  uint8_t timerNumber = 0; // Timer0 will be used to control the right motor
  uint16_t prescaleDivider = 80; // Timer0 will use a presaler (divider) of 80 so that each interrupt occur at 1us
  bool countUp = true; // Timer0 will count up not down
  rightMotorTimer = timerBegin(timerNumber, prescaleDivider, countUp); // Set Timer0 configuration
  bool intOnEdge = true; // Interrupt on rising edge of Timer0 signal
  timerAttachInterrupt(rightMotorTimer, &rightMotorTimerISR, intOnEdge); // Attach ISR to Timer0
  bool autoReload = true; // Shoud the ISR timer reload after it runs
  timerAlarmWrite(rightMotorTimer, motorISRus, autoReload); // Set up conditions to call ISR
  timerAlarmEnable(rightMotorTimer); // Enable ISR
  // Set up left motor driver ISR
  AMDP_PRINTLN("<setupDriverMotors> Configure timer1 to control the left motor");  
  timerNumber = 1; // Timer1 will be used to control the right motor
  prescaleDivider = 80; // Timer1 will use a presaler (divider) of 80 so that each interrupt occur at 1us
  countUp = true; // Timer1 will count up not down
  leftMotorTimer = timerBegin(timerNumber, prescaleDivider, countUp); // Set Timer1 configuration
  timerAttachInterrupt(leftMotorTimer, &leftMotorTimerISR, intOnEdge); // Attach ISR to Timer1
  autoReload = true; // Shoud the ISR timer reload after it runs
  timerAlarmWrite(leftMotorTimer, motorISRus, autoReload); // Set up conditions to call ISR
  timerAlarmEnable(leftMotorTimer); // Enable ISR
  // Attach interrupts to track DVR8825 faults
  AMDP_PRINTLN("<setupDriverMotors> Monitor left & right DRV8825 drivers for faults");  
  attachInterrupt(gp_DRV1_FAULT, rightDRV8825fault, FALLING);  
  attachInterrupt(gp_DRV2_FAULT, leftDRV8825fault, FALLING);
} //setupDriverMotors()

/** 
 * @brief Standard set up routine for Arduino programs 
 */
void setup() 
{
  Wire.begin(gp_I2C_IMU_SDA,gp_I2C_IMU_SCL,I2C_bus1_speed);  
  Serial.begin(115200); // Open a serial connection at 115200bps
  while (! Serial); // Wait for Serial port to be ready
  Serial.println(F("<setup> Start of setup"));
  cfgByMAC(); // Use the devices MAC address to make specific configuration settings
  setupLED(); // Set up the LED that the loop() flashes
  setupFreeRTOStimers(); //  User timer based FreeRTOS threads to manage a number of asynchronous tasks
  setupMQTT(); // Set up MQTT communication
  setupWiFi(); // Set up WiFi communication
  setupOLED(); // Setup OLED communication
  setupIMU(); // Set up IMU communication
  setupDriverMotors(); // Set up the Stepper motors used to drive the robot motion
  Serial.println(F("<setup> End of setup"));
  goOLED = millis() + tmrOLED; // Reset OLED update counter
  goLED = millis() + tmrLED; // Reset LED flashing counter
  goIMU = millis() + tmrIMU; // Reset IMU update counter
  goMETADATA = millis() + tmrMETADATA; // Reset IMU update counter
  cntLoop = 0; // Loop counter does not interact with timer ISR so no muxing is required 
} //setup()

/**
 * @brief Standard looping routine for Arduino programs
 */
void loop() 
{
  cntLoop ++; // Increment loop() counter
  if((millis() >= goIMU) && mpuInterrupt == true) readIMU(); // Update the OLED with data
  if(millis() >= goOLED) updateOLED(ypr[2]); // Update the OLED with data. Use ypr[0], ypr[1], ypr[2] depending on circuit orientation
  if(millis() >= goLED) updateLED(); // Update the OLED with data
  if(millis() >= goMETADATA) updateMetaData(); // Send data to serial terminal
} //loop()