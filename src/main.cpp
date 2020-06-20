/*************************************************************************************************************************************
 * @file main.cpp
 * @author va3wam
 * @brief Run tests to determine the fastest times we can use on TWIPe for OLED and MQTT updates of data
 * @version 0.0.16
 * @date 2020-04-25
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ----------------------------------------------------------------------------------------------------------------
 * 0.0.19  2020-06-18 DE: bug fix to motor speed change smoothing code
 *                      -make wheels respond to change in direction more quickly by aborting current step thats in the wrong direction
 *                      -take "Ang: " label out of right OLED display
 * 0.0.18  2020-06-17 DE: add MQTT command targetAngle=f   where float f is a number of degrees. -1 is one degree back of vertical
 *                      -add telemetry support for pid_i_gain, pid_d_gain, targetAngle
 *                      -move telemetry for control params & data titles from bs.active state to bs.awake, before we get busy
 *                      -added MQTT remote control of motor speed controls with commands motor_int_slow=n and motor_int_fast=n
 *                      -added MQTT command to set angle when balancing starts, around 2 degrees.  active_angle=<float>
 *                      -new remotely controllable parameters are supported in spreadsheet version 10
 * 0.0.17  2020-06-17 AM: Added MQTT commands to set pid_p_gain, pid_i_gain, pid_d_gain and smoother remotely to help speed up PID tuning
 * 0.0.16  2020-06-15 DE: make OLED display optional, controlled by bool OLED_enable
 *                     -fix calculation of OldBalByAng telemetry time interval
 *                     -restructure loop() with else if's, so only one routine ever runs before next goIMU check
 *                     -add execution time of left and right OLED updates to telemetry. Yup - they're huge.
 *                     -move writing of netinfo to left eye into setup(), since info is static, and doesn't need refreshes
 *                       -subsequently added it when entering bs_awake, to get all values to display
 *                     -add execution time of updateMetadata() to telemetry
 *                     -remove display of MotorInt and PID from right eye, to see what execution time reduction we get
 *                     -add execution time for updateMetadata() to telemetry
 *                     -play with adjusting pid_p_gain & watching telemetry
 *                     -removed serial I/O from onMQTTpublish(), which runs at a high frequency
 *                     -add pid_i_gain and pid_d_gain parameters for controlling PID algorithm
 *                     -zero telemetry values after publication, so leftovers don't get published if routine doesn't run
 *                     - in tm_MQpubCnt, count executions of onMQTTpublish() between balance telemetry publishes
 *                     -fix telemetry titles for R.O.time & MQpubCnt (they were swapped)
 * 0.0.15  2020-06-13 DE: increase angle reaction time by reducing tmrIMU to 20 milliseconds
 *                     - move tmrIMU reset to loop(), rather than at end of readIMU for more accuracy
 *                     - remove Balance.state from balance telemetry - not useful
 *                     - parameterize motor wheel direction differences between bots
 *                     - add runFlags so telemetry can show what routines have run recently
 *                     - recode balance telemetry to send calculated intervals rather than timestamps
 * 0.0.14  2020-06-12 DE: reverse direction of wheels by multiplying motorInt by -1
 *                    - add support for left OLED, and use it to display current setup() stage, and then network info
 *                    - remove //de markers, except where code changes may be needed
 *                     - move some (not all) balanceByAngle params and variables into structs
 *                     - enhanced balance MQTT data, adding general purpose timestamps telMillis1 - 5
 *                     - added MQTT data title publishing when balance state enters bs_active, ctntroled by balTelMsg.needTitles
 *                     - adding MQTT publish of balance control params: pid gain, hi & lo speeds, smoother, tmrISU, once bot is bs_active
 * 0.0.13  2020-06-02 DE: jump the Version number to sync up with master on Git
 *                    - trim so source lines don't exceed 140 characters
 *                    - rename RobotBalance struct to Balance. Hmm, holding off on further changes like this since there seems to be a
 *                      standard of prefixing the prefix with "robot". 
 *                    - add Balance.state to track what part of balancing process we're in, and checkBalanceState() called from loop()
 *                    - add second balancing method, by angle, with control by Balance.method variable, and 2 part motor ISR's
 *                       key new routine is balanceByAngle() called from loop()
 *                    - add display of motor "speed" variable to OLED, and current PID value
 *                    - add smoothing to motor speed changes
 *                    - add fallback from balance state awake to sleep, then fix it
 *                    - correct topic used for MQTT balance data in balanceByAngle()
 * 0.0.8   2020-05-29 DE: update my bot's calibraton data and correct printout
 *                    add three methods to calculate tile angle:
 *                    1)non-DMP, using methods from 2 websites:
 *                      https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MP
 *                      https://hester.mtholyoke.edu/idesign/SensorTilt.html
 *                    2)DMP, using one of the YPR values it provides, with a gimbal lock risk
 *                    3)DMP, using quaternions to rotate original "up" vector (0,0,1)
 *                    -add a visible separator line of equal signs before each routine definition
 *                    -add another debug print command: AMDP_PRINTF(x,y)
 *                   -left in the mpu.Calibrate... calls, although they're absent in Rowberg's lates
 *                   -require the MPU6050 library with the fix on line 2764, by renaming it to MPU60
 *                     MPU6050-6Axis_MotionApps_V6_12 as well, because it references MPU6050.h
 *                   -reformat #includes to clarify which libraries we created ourselves, and make l
 *                   -remove all support for DMP interrupts, which latest Rowberg code doesn't need
 *                   -select DMP's YPR[2] as best tilt angle to use, in variable tilt, and trim code 
 * 0.0.12  2020-06-01 AM: Corrected wheel diameter value. Moved min/max PWM data to new metadata structure. Removed all the 
 *                        portENTER_CRITICAL_ISR commands and associated portMUX_TYPE variabes as these are only used to protect 
 *                        FREERTOS threads which we do not use at the moment. We can use nointerrupt() and interrupt() instead if
 *                        needed. Also added stepMotor call back in. At this point no tuning has been done but all the bits needed
 *                        to balance are here. 
 * 0.0.11  2020-05-31 AM: moved calcBalanceParmeters(ypr[2]) to loop() from readIMU(). Also added checkTiltToActivateMotors()
 *                    to loop() so robot now enables and disables motors based on the tilt of the robot 
 * 0.0.10  2020-05-31 AM: Added portENTER_CRITICAL(&balanceMUX) and portEXIT_CRITICAL(&balanceMUX) to calcBalanceParameters. Added 
 *                    proper header and print labels to processWifiEvent(). 
 * 0.0.9   2020-05-31 AM: Added messaging structure. Removed distance and odometer from motor structure. Changed metadata messaging
 *                    to use new message structure. Replaced formatBalanceData() with calcBalanceParmeters(). Removed MQTT control
 *                    of motors. Replaced local variables in calcBalanceParameters() with robotBalance structure.
 * 0.0.8   2020-05-29 AM: Fixed up code to do quick and dirty balancing
 * 0.0.7   2020-05-29 AM: Updated the IMU calibration data for Andrew's robot and added a call to setbalanceDistance() from readIMU() 
 *                        that does not seem  to work.
 * 0.0.6   2020-05-23 DAE: make Wifi startup more robust by calling connectToWifi after certain errors
 *                    Make Wifi event handler just store the event for loop() to process at background level
 *                    Temporarily disable timer that calls connectToWifi() in handling of STA_DISCONNECTED event
 *                      to avoid it running asynchronously to other code. It also needs same retry logic that
 *                      connectToNetwork() has.
 * 0.0.5   2020-05-16 Reset odometer value in setBalanceDistance()
 * 0.0.4   2020-05-14 Added structure to track physical attributes of robot as well as the function setBalanceDistance() which accepts
 *                    and angle and uses it to set the motor tripDistance in steps to try and balance the robot. 
 * 0.0.3   2020-05-12 Added manual motor control via MQTT
 * 0.0.2   2020-04-27 Special trimmed down build that just doe IMU output to the serial port
 * 0.0.1   2020-03-13 Program created
 *************************************************************************************************************************************/
// TODO Add boot sequence that 1) checks Flash for config, or 2) asks MQTT for config, or 3) Uses default values in include file
// TODO Add MQTT topic which is updated at boot up
// TODO Fix bug where sometimes MQTT commands do not terminate and the command goes forever

// Arduino libraries
#include <Arduino.h>                                // Arduino Core for ESP32 
// from https://github.com/espressif/arduino-esp32. Comes with Platform.io
#include <WiFi.h>                                   // Required to connect to WiFi network. 
// Comes with Platform.io
#include <I2Cdev.h>                                 // For MPU6050 
// from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/I2Cdev/I2Cdev.h
#include <MPU6050_6Axis_MotionApps_V6_12-fix2764.h> // for MPU6050. edited to refer to our MPU6050-fix2764.h
// from https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050_6Axis_MotionApps_V6_12.h
#include <Wire.h>                                   // Required for I2C communication. 
// Comes with Platform.io
#include <SSD1306.h>                                // For OLED 
// from https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <MPU6050-fix2764.h>                        // for MPU6050
// require the version that has fix for misplaced parenthesis at line 2764
#include <huzzah32_pins.h>                          // Defines our usage of the GPIO pins for Adafruit Huzzah32 dev board
// our own creation
#include <i2c_metadata.h>                           // Defines all I2C related info including device addresses, bus pins and bus speeds
// our own creation
#include <known_networks.h>                         // Defines Access points and passwords that the robot can scan for and connect to
// our own creation
#include <AsyncMqttClient.h> // for Message Queuing Telemetry Support
// from https://github.com/marvinroger/async-mqtt-client

// FreeRTOS libraries  
#include "freertos/FreeRTOS.h"      // Required for threads that control wifi and mqtt connections
// Comes with Platform.io ?
#include "freertos/timers.h"        // Required for xTimerCreate function used for controlling wifi and mqtt connections
// Comes with Platform.io ?
#include <freertos/event_groups.h>  // Required to use the FreeRTOS function xEventGroupSetBits. Used for motor driver control
// Comes with Platform.io ?
#include <freertos/queue.h>         // Required to use FreeRTOS the function uxQueueMessagesWaiting. Used for motor driver control
// Comes with Platform.io ?

// Precompiler directives for debug output 
#define DEBUG true                  // Turn debug tracing on/off
#define DMP_TRACE false             // Set to TRUE or FALSE to toggle DMP memory read/write activity

// Create debug macros that mirror the standard c++ print functions. Use the pre-processor variable 
#if DEBUG == true
    #define AMDP_PRINT(x) Serial.print(x)
    #define AMDP_PRINTLN(x) Serial.println(x)
    #define AMDP_PRINTF(x, y) Serial.printf(x, y)   // allow more concise debug output
#else // Map macros to "do nothing" commands so that when is not TRUE these commands do nothing
    #define AMDP_PRINT(x)
    #define AMDP_PRINTLN(x)
    #define AMDP_PRINTF(x, y)
#endif


// Define which core the Arduino environment is running on
#if CONFIG_FREERTOS_UNICORE    // If this is an SOC with only 1 core
#define ARDUINO_RUNNING_CORE 0 // Arduino is running on that one core
#else                          // If this is an SOC with more than one core (2 is the ony other option at ths point)
#define ARDUINO_RUNNING_CORE 1 // Arduino is running on the second core
#endif


typedef struct
{
  float heightCOM = 0;                 // Height from ground to Center Of  Mass of robot in inches
  float wheelDiameter = 3.937008;      // Diameter of wheels in inches. https://www.robotshop.com/en/100mm-diameter-wheel-5mm-hub.html
  float wheelCircumference;            // Diameter x pi
  float distancePerStep = 0;           // Distance travelled per step of motor in inches
  int16_t XGyroOffset;                 // Gyroscope x axis (Roll)
  int16_t YGyroOffset;                 // Gyroscope y axis (Pitch)
  int16_t ZGyroOffset;                 // Gyroscope z axis (Yaw)
  int16_t XAccelOffset;                // Accelerometer x axis
  int16_t YAccelOffset;                // Accelerometer y axis
  int16_t ZAccelOffset;                // Accelerometer z axis
} robotAttributes;                     // Structure of attributes of the robot
static volatile robotAttributes robot; // Object of attributes of the robot
#define STATE_STAND_GROUND 0
#define STATE_MOVE_FORWARD 1
#define STATE_MOVE_BACKWARD 2
#define STATE_TURN_RIGHT 3
#define STATE_TURN_LEFT 4
#define STATE_PARAMETER_UNUSED 0
#define STATE_TEST_MOTOR 99
typedef struct
{
  int activity = STATE_STAND_GROUND;      // The current objective that the robot is pursuing
  int parameter = STATE_PARAMETER_UNUSED; // A parameter used by some modes such as turn left and right
  float targetDistance = 0;               // Target distance robot wants to maintain
  float targetAngleDegrees = 0;          // Target angle the robot wants to maintain to achieve the target distance. 0 = stay vertical
} state;                                  // Structure for stepper motors that drive the robot
static volatile state robotState;         // Object of states the robot is in pursuing vaious goals

// Define OLED constants, classes and global variables
SSD1306 rightOLED(rightOLED_I2C_ADD, gp_I2C_LCD_SDA, gp_I2C_LCD_SCL);
SSD1306 leftOLED(leftOLED_I2C_ADD, gp_I2C_LCD_SDA, gp_I2C_LCD_SCL);  // same I2C bus, but different address
bool OLED_enable = true;                  // allow disabling OLED for performance troubleshooting

// Define LED constants, classes and global variables
bool blinkState = false;

// Define MPU6050 constants, classes and global variables
// Note that we are using Yaw/Pitch/Roll which might suffer from gimble lock http://en.wikipedia.org/wiki/Gimbal_lock
MPU6050 mpu;            // GY521 default I2C address
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Define global WiFi network information
const char *mySSID = "NOTHING";
const char *myPassword = "NOTHING";
String myMACaddress;               // MAC address of this SOC. Used to uniquely identify this robot
String myIPAddress;                // IP address of the SOC.
String myAccessPoint;              // WiFi Access Point that we managed to connected to
String myHostName;                 // Name by which we are known by the Access Point
String myHostNameSuffix = "Twipe"; // Suffix to add to WiFi host name for this robot
WiFiClient client;                 // Create an ESP32 WiFiClient class to connect to the MQTT server
String tmpHostNameVar;             // Hold WiFi host name created in this function

TimerHandle_t wifiReconnectTimer;  // Reference to FreeRTOS timer used for restarting wifi
int wifiCurrConAttemptsCnt = 0;    // Number of Acess Point connection attempts made during current connection effort
int WifiLastEvent = -1;            // last seen Wifi event, that needs to be handled in loop()
                                   // 0 = Wifi ready, 4 = connected, 5 = disconnected, 7 = got IP address

char const* wifiSt[]               // WiFi status code translations
{ "WL_IDLE_STATUS","WL_IDLE_STATUS","WL_SCAN_COMPLETED","WL_CONNECTED","WL_CONNECT_FAILED","WL_CONNECTION_LOST","WL_DISCONNECTED"
};

char const* wifiEv[]               // WiFi event number translations
{"WiFi ready","AP scan done","station start","station stop","connected to AP","disconnected from AP","auth mode changed","got IP","lost IP"
};

// Define MQTT constants, classes and global variables.
// Note that the MQTT broker used for testing this is Mosquitto running on a Raspberry Pi
// Note sends balance telemetry data to <device name><telemetry/balance>
// Note listens for commands on <device name><commands>
const char* MQTT_BROKER_IP = "not-assigned";   // Need to make this a fixed IP address
#define MQTT_BROKER_PORT 1883             // Use 8883 for SSL
#define MQTT_USERNAME "NULL"              // Not used at this time. To do: secure MQTT broker
#define MQTT_KEY "NULL"                   // Not used at this time. To do: secure MQTT broker
#define MQTT_IN_CMD "/commands"           // Topic branch for incoming remote commands
#define MQTT_TEL_BAL "/telemetry/balance" // Topic tree for outgoing balance telemetry data
#define MQTT_METADATA "/metadata"         // Topic tree for outgoing metadata about the robot
#define QOS1 1                            // Quality of service level 1 ensures one time delivery
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
String cmdTopicMQTT = "NOTHING"; // Full path to incoming command topic from MQTT broker
String balTopicMQTT = "NOTHING"; // Full path to outgoing balance telemetry topic to MQTT broker
String metTopicMQTT = "NOTHING"; // Full path to outgoing metadata topic to MQTT broker


// Define global motor control variables and structures. Also define pointers and muxing for multitasking motors via ISRs
#define motorISRus 20               // Number of microseconds between motor ISR calls
#define RIGHT_MOTOR 0               // Index value of right motor array
#define LEFT_MOTOR 1                // Index value of right motor array
hw_timer_t *rightMotorTimer = NULL; // Pointer to right motor ISR
hw_timer_t *leftMotorTimer = NULL;  // Pointer to left motor ISR
//portMUX_TYPE leftMotorTimerMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate left motor variable access for ISR & main thread
//portMUX_TYPE rightMotorTimerMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate right motor variable access for ISR & main thread
//portMUX_TYPE leftDRVMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate left DVR8825 fault err var. access for ISR & main thread
//portMUX_TYPE rightDRVMux = portMUX_INITIALIZER_UNLOCKED; // Mux to coordinate right DVR8825 fault err var. access for ISR & main thread
typedef struct
{
  long interruptCounter;                      // Counter for step signals to the DRV8825 motor driver
  int minSpeed = 300;                         // Minimum speed the motor should run at to be effective
  int speedRange = 300;                       // Range of speed the motor is effective at. Add to minSpeed to get max speed
  int interval = 600;                         // Delay time (microseconds) used for motor PWM. smaller number -> higher motor speed
  int stepsPerRev = 200;                      // How many steps it takes to do a full 360 degree rotation
  int directionMod = 1;                       // controls if motor direction needs to be reversed base on motor hardware
} motorControl;                               // Structure for stepper motors that drive the robot
static volatile motorControl stepperMotor[2]; // Define an array of 2 motors. 0 = right motor, 1 = left motor

// Define global control variables.
#define NUMBER_OF_MILLI_DIGITS 10 // Millis() uses unsigned longs (32 bit). Max value is 10 digits (4294967296ms or 49 days, 17 hours)
#define tmrIMU 20                 // Milliseconds to wait between reading data to IMU over I2C, and doing balancing calculations
#define tmrOLED 200               // Milliseconds to wait between sending data to OLED over I2C
#define tmrMETADATA 1000          // Milliseconds to wait between sending data to serial port
#define tmrLED 1000 / 2           // Milliseconds to wait between flashes of LED (turn on / off twice in this time)
uint32_t cntLoop = 0;             // Track how many times loop() has iterated
int goIMU = 0;                    // Target time for next read of IMU data
int goOLED = 0;                   // Target time for next OLED update
int goMETADATA = 0;               // Target time for next serial port
int goLED = 0;                    // Target time for next toggle of LED
// multi-purpose timestamp holders for telemetry purposes
unsigned long telMilli1;          // timestamp used for telemetry reporting
unsigned long holdMilli1;         // need to keep last value to calculate delta time for goIMU calls
unsigned long telMilli2;          // timestamp used for telemetry reporting
unsigned long telMilli3;          // timestamp used for telemetry reporting
unsigned long telMilli4;          // timestamp used for telemetry reporting
unsigned long holdMilli4;         // need to retain old value for balanceByAngle telemetry calculation
unsigned long telMilli5;          // timestamp used for telemetry reporting

unsigned long tm_IMUdelta;        // telemetry value: how long between goIMU calls. should be tmrIMU
unsigned long tm_readFIFO;        // telemetry value: how long the mpu.dmpGetCurrentFIFOPacket(fifoBuffer) execution took
unsigned long tm_dmpGet;          // telemetry value: how long the dmpGet* calls after above call took
unsigned long tm_allReadIMU;      // telemetry value: how long the readIMU execution took
unsigned long tm_OldbalByAng;     // telemetry value: how long the PREVIOUS balanceByAngle took
unsigned long tm_ROLEDtime;       // telemetry measure: time spent in right OLED update
unsigned long tm_LOLEDtime;       // telemetry measure: time spent in left OLED update 
unsigned long tm_uMDtime;         // telemetry measure: time spent in updateMetadata()
int tm_MQpubCnt = 0;              // telemetry measure: count of onMQTTpublish() executions

unsigned long runFlagWord;        // telemetry word with bit coded flags indicating if a routine has run since last telemetry
                                  // the indicated routine has runbit(n); at its beginning, where n is it's bit number, 0 - 31
                                  // the runFlagWord is cleared after each balance telemetry publish
                                  
#define runbit(x)   runFlagWord  |= 1 << x;
//             NOTES  (see below)
//             -----
// runbit(0)    1     IRAM_ATTR leftDRV8825fault
// runbit(1)    1     IRAM_ATTR rightDRV8825fault
// runbit(2)    1  T  connectToWifi
// runbit(3)___ 1  T  connectToMqtt
// runbit(4)    1  W  WiFiEvent
// runbit(5)    1     processWifiEvent
// runbit(6)    1  M  onMqttConnect
// runbit(7)___ 1  M  onMqttDisconnect
// runbit(8)    1  M  onMqttSubscribe
// runbit(9)    1  M  onMqttUnsubscribe
// runbit(10)   1  M  onMqttMessage
// runbit(11)___1  M  onMqttPublish
//              0     connectToNetwork
//              0     scanNetworks
//              0     printBinary
// runbit(12)   1     publishMQTT
// runbit(13)   1     stepMotor
// runbit(14)   1     IRAM_ATTR rightMotorTimerISR
// runbit(15)___1     IRAM_ATTR leftMotorTimerISR
// runbit(16)   1     calcBalanceParmeters
//              0     balanceByAngle
// runbit(17)   1     updateMetaData
// runbit(18)   1     updateLeftOLEDNetInfo
// runbit(19)___1     updateRightOLED
// runbit(20)   1     updateLeftOLED
// runbit(21)   1  S  setupWiFi
// runbit(22)   1     subscribed_callback
// runbit(23)___1  S  setupMQTT
//              0     setupLED
//              0     setupOLED
//              0     setupIMU
//              0     cfgByMAC
// runbit(24)   1     updateLED
// runbit(25)   1  S  setupFreeRTOStimers
// runbit(26)   1  S  setupDriverMotors
// runbit(27)___1     checkBalanceState
// runbit(28)   1  S  setRobotObjective
//              0     setup
//              0     loop
//
// NOTES:
//  0  excluded - doesn't have associated runbit in runFlagword
//  1  included - routine has a call to runbit(n) at its beginning
//  T  timer task - task is initiated by an RTOS timer, and runs asynchronous to loop()
//  W  WiFi Event - task is initiated by WiFi activity, and runs asynchronous to loop()
//  S  Startup only - task runs within startup() only, and doesn't need runbit tracking

#define TARGET_CONSOLE 0
#define TARGET_MQTT 1
typedef struct
{
  boolean active = false;
  boolean destination = TARGET_CONSOLE;
  String message = "";
  bool needTitles = false;                  // whether or not data title MQTT msg should be sent before next MQTT msg
} messageControl;                           // Structure for handling messaging for key objects
static volatile messageControl baltelMsg;   // Object that contains details for controlling balance telemetry messaging
static volatile messageControl metadataMsg; // Object that contains details for controlling metadata messaging
//portMUX_TYPE messageMUX = portMUX_INITIALIZER_UNLOCKED; // Syncronize message variables between ISR and loop()
typedef struct
{
  float tilt = 0;                            // forward/backward angle of robot, in degrees, positive is leaning forward, 0 is vertical
  int method;                                // are we using catchup distance balancing method, or angle based PID (see defs below)
  int state = 0;                             // state within balancing process. see state definitions below, as in bs_start
  float activeAngle = 1;                     // how close to vertical, in degrees, before you start balancing attempt, & bs_active      
  float angleRadians = 0;                    // Robot tilt angle in radians
  float angleDegrees = 0;                    // Robot tilt angle in degrees - a copy of tilt for now
  //de  need to understand following line, and convert it to reflect vertical = 0 degrees
  float angleTargetRadians = 1.5707961;      // Target angle robot wants to be at in radians. 1.5707961 is standing upright
  float angleTargetDegrees = 0;             // Target angle robot wants to be at in degrees. 0 is standing upright
  float maxAngleMotorActiveDegrees = 30;     // Maximum angle the robot can lean at before motors shut off
  float centreOfMassError = robot.heightCOM; // Distance in inches robot's Centre Of Mass (COM) is away from target
  float distancePercentage;                  // Percentage of COM height away from target
  int steps;                                 // Number of steps that it will take to get to target angle
} balanceControl;                            // Structure for handling robot balancing calculations
volatile balanceControl Balance;             // Object for calculating robot balance

// values for Balance.state
#define bs_sleep 0     // inactive. from lying on back until 30 degrees from vertical
#define bs_awake 1     // within 30 degrees of initial vertical, but still not active
#define bs_active 2    // has hit vertical, and is now trying to balance

// values for Balance.method
#define bm_catchup 1   // method based on catchup distance that would pull wheels under the center of mass
#define bm_angle 2     // method based on applying correction based in PID applied to angle difference from vertical
#define bm_initialMethod bm_angle    // use this balancing method to start, initialized in setupIMU

volatile int throttle_Lcounter, throttle_Rcounter;        // working counter of timer ticks in motor controller ISRs
volatile int throttle_Llimit, throttle_Rlimit;            // limit that determines step length in timer ticks
volatile int throttle_Lsetting, throttle_Rsetting;        // value for limit calculated in BalancceByAngle()
float pid;                                                  // global so updateOLED() can find it

float pid_p_gain = 200;       //de multiplier for the P part of PID
float pid_i_gain = 0;        //de multiplier for the I part of PID
float pid_d_gain = 0;        //de multiplier for the D part of PID
int motorInt;                // motor speed, i.e. interval between steps in timer ticks
int bot_slow =550;           //de  need to fit these into structures, but quick & dirty now
int bot_fast = 350;
float smoother = 0 ;         // smooth changes in speed by using new = old + smoother * (new - old). smoother=0 > disable smoothing  
int lastSpeed = 0;           // memory for above method using smoother

//portMUX_TYPE balanceMUX = portMUX_INITIALIZER_UNLOCKED; // Syncronize balance variables between ISR and loop()

// Define global metadata variables. Used too understand the state of the robot, its peripherals and its environment.
typedef struct
{
  int wifiConAttemptsCnt = 0;    // Track the number of over all attempts made to connect to the WiFi Access Point
  int mqttConAttemptsCnt = 0;    // Track the number of attempts made to connect to the MQTT broker
  int dmpFifoDataMissingCnt = 0; // Track how many times the FIFO pin goes high but the buffer is empty
  int dmpFifoDataPresentCnt = 0; // Track how many times the FIFO pin goes high and there is data in the buffer
  int wifiDropCnt = 0;           // Track how many times connection to the WiFi network has occurred
  int mqttDropCnt = 0;           // Track how many times connection to the MQTT server is lost
  int unknownCmdCnt = 0;         // Track how many unknown command have been received
  int leftDRVfault = 0;          // Track how many times the left DVR8825 motor driver signals a fault
  int rightDRVfault = 0;         // Track how many times the right DVR8825 motor driver signals a fault
  //TODO Put datapoint below to use
  long riseTimeMax = 0;                     // Most microseconds it took for the signal rise event to happen
  long riseTimeMin = 0;                     // Least microseconds it took for the signal rise event to happen
  long fallTimeMax = 0;                     // Most microseconds it took for the signal fall event to happen
  long fallTimeMin = 0;                     // Least microseconds it took for the signal fall event to happen
  int delayTimeMax = 0;                     // Most microseconds it took for the delay time event to happen
  int delayTimeMin = 0;                     // Least microseconds it took for the delay time event to happen
} metadataStructure;                        //Structure for tracking key points of interest regarding robot performance
static volatile metadataStructure metadata; // Object for tracking metadata about robot performance
// Define flags that are used to track what devices/functions are verified working after start up. Initilize false.
boolean leftOLED_detected = false;
boolean rightOLED_detected = false;
boolean LCD_detected = false;
boolean MPU6050_detected = false;
boolean wifi_connected = false;

// ISR dmpDataReady(), once used for IMU DMP interrupts is now unneeded and has been removed

// TODO understand the use of IRAM

/** 
 * @brief Strips the colons off the MAC address of this device
 * @return String
 */
String formatMAC()
{
  String mac;
  AMDP_PRINTLN("<formatMAC> Removing colons from MAC address");
  mac = WiFi.macAddress(); // Get MAC address of this SOC
  mac.remove(2, 1);        // Remove first colon from MAC address
  mac.remove(4, 1);        // Remove second colon from MAC address
  mac.remove(6, 1);        // Remove third colon from MAC address
  mac.remove(8, 1);        // Remove forth colon from MAC address
  mac.remove(10, 1);       // Remove fifth colon from MAC address
  AMDP_PRINT("<formatMAC> Formatted MAC address without colons = ");
  AMDP_PRINTLN(mac);
  return mac;
} //formatMAC()

/**
 * @brief ISR for left DRV8825 fault condition
===================================================================================================*/
void IRAM_ATTR leftDRV8825fault()
{
  runbit(0) ;
  //  portENTER_CRITICAL_ISR(&leftDRVMux);

  metadata.leftDRVfault++;
  //  portEXIT_CRITICAL_ISR(&leftDRVMux);
} // leftDRV8825fault()

/**
 * @brief ISR for right DRV8825 fault condition
 * 
=================================================================================================== */
void IRAM_ATTR rightDRV8825fault()
{
  runbit(1) ;
  //  portENTER_CRITICAL_ISR(&rightDRVMux);
  metadata.rightDRVfault++;
  //  portEXIT_CRITICAL_ISR(&rightDRVMux);
} // rightDRV8825fault()

/** 
 * @brief This function returns a String version of the local IP address
 */
String ipToString(IPAddress ip)
{
  AMDP_PRINTLN("<ipToString> Converting IP address to String.");
  String s = "";
  for (int i = 0; i < 4; i++)
  {
    s += i ? "." + String(ip[i]) : String(ip[i]);
  } //for
  AMDP_PRINT("<ipToString> IP Address = ");
  AMDP_PRINTLN(s);
  return s;
} //ipToString()

/**
 * @brief Connect to WiFi Access Point 
=================================================================================================== */
void connectToWifi()
{
  runbit(2) ;
  metadata.wifiConAttemptsCnt++; // Increment the number of attempts made to connect to the Access Point
  AMDP_PRINT("<connectToWiFi> Attempt #");
  AMDP_PRINT(metadata.wifiConAttemptsCnt);
  AMDP_PRINTLN(" to connect to a WiFi Access Point");
  WiFi.begin(mySSID, myPassword);
} //connectToWifi()

/**
 * @brief Connect to MQTT broker
 * @note MQTT Spec: https://docs.oasis-open.org/mqtt/mqtt/v3.1.1/os/mqtt-v3.1.1-os.html#_Toc398718063
 * @note Reference for MQTT comments in this code: https://www.hivemq.com/mqtt-essentials/
=================================================================================================== */
void connectToMqtt()
{
  runbit(3) ;
  AMDP_PRINTLN("<connectToMqtt> Connecting to MQTT...");
  mqttClient.connect();
  metadata.mqttConAttemptsCnt++; // Increment the number of attempts made to connect to the MQTT broker
} //connectToMqtt()

/**
 * @brief Keeps track of the last WiFi event that occurred and prints it out
 * @param event WiFi event that caused this function to be called
 * @note Called from WiFi event handler which is needed  to process MQTT messages for some reason
 
 * # WiFi Event Handling
 * All Wifi events are processed by the WiFiEvent method. A list of the events appears in the table below.
 * Events preceded by a ">" are common, and have been seen in TWIPe debug logs
 * 
 * ## Table of WiFi Events
 * | Return Code | Constant Directive | Description                                                      |  
 * |:-----------:|:--------------------------------------------------------------------------------------|
 * | 0  | SYSTEM_EVENT_WIFI_READY | WiFi ready |
 * | 1  | SYSTEM_EVENT_SCAN_DONE | finish scanning AP |
 * | 2  | SYSTEM_EVENT_STA_START | station start |
 * | 3  | SYSTEM_EVENT_STA_STOP | station stop |
 * > 4  | SYSTEM_EVENT_STA_CONNECTED | station connected to AP |
 * > 5  | SYSTEM_EVENT_STA_DISCONNECTED | station disconnected from AP |
 * | 6  | SYSTEM_EVENT_STA_AUTHMODE_CHANGE | the auth mode of AP connected by ESP32 station changed |
 * > 7  | SYSTEM_EVENT_STA_GOT_IP | station got IP from connected AP |
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
 * 
 * 
 * Values for WifiStatus()
 * Events preceded by a ">" are common, and have been seen in TWIPe debug logs
 * ---------------------------
 * |255| WL_NO_SHIELD	255
 * > 0 | WL_IDLE_STATUS	0
 * > 1 | WL_NO_SSID_AVAIL	1
 * | 2 | WL_SCAN_COMPLETED	2
 * > 3 | WL_CONNECTED	3
 * > 4 | WL_CONNECT_FAILED	4
 * > 5 | WL_CONNECTION_LOST	5
 * > 6 | WL_DISCONNECTED	6
=================================================================================================== */
void WiFiEvent(WiFiEvent_t event)
{
  runbit(4) ;
  AMDP_PRINT("<WifiEvent> saw event number: ");
  AMDP_PRINT(event);
  AMDP_PRINTLN(String(" = ") + wifiEv[event]);
  if (WifiLastEvent != -1)
  {
    AMDP_PRINT("<WiFiEvent> ********* Overwrote an unprocessed event *********  ");
    AMDP_PRINT(wifiEv[WifiLastEvent]);
    AMDP_PRINT(" was replaced by: ");
    AMDP_PRINTLN(wifiEv[event]);
  }
  WifiLastEvent = event; // remember what event it was, and signal loop() to process it
} //WiFiEvent()

/**
 * @brief Actually handles WiFi events using the last known wifi event that was set in WiFiEvent()
 * @note Called from loop()
=================================================================================================== */
void processWifiEvent() // called fron loop() to handle event ID stored in WifiLastEvent
{
  runbit(5) ;
  int event = WifiLastEvent; // retrieve last event that occurred
  WifiLastEvent = -1;        // and say that we've processed it
  AMDP_PRINT("<processWiFiEvent> event:");
  AMDP_PRINTLN(event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_CONNECTED:
  {
    AMDP_PRINTLN("<processWiFiEvent> Event 4 = Got connected to Access Point");
    break;
  } //case
  case SYSTEM_EVENT_STA_DISCONNECTED:
  {
    AMDP_PRINTLN("<processWiFiEvent> Lost WiFi connection");
    //      int blockTime  = 10; // https://www.freertos.org/FreeRTOS-timers-xTimerStart.html
    //      xTimerStop(mqttReconnectTimer, blockTime);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi.
                                                        // Disconnect triggers new connect atempt
    //      xTimerStart(wifiReconnectTimer, blockTime); // Activate wifi timer (which only runs 1 time)
    wifi_connected = false;
    metadata.wifiDropCnt++; // Increment the number of network drops that have occured
    break;
  } //case
  case SYSTEM_EVENT_STA_GOT_IP:
  {
    AMDP_PRINT("<processWiFiEvent> Event 7 = Got IP address. That address is: ");
    AMDP_PRINTLN(WiFi.localIP());
    myIPAddress = ipToString(WiFi.localIP());
    myAccessPoint = WiFi.SSID();
    tmpHostNameVar = myHostNameSuffix + myMACaddress;
    WiFi.setHostname((char *)tmpHostNameVar.c_str());
    myHostName = WiFi.getHostname();
    Serial.print("<processWiFiEvent> Network connection attempt #");
    Serial.print(metadata.wifiConAttemptsCnt);
    Serial.print(" SUCCESSFUL after this many tries: ");
    Serial.println(wifiCurrConAttemptsCnt);
    Serial.println("<processWiFiEvent> Network information is as follows...");
    Serial.print("<processWiFiEvent> - Access Point Robot is connected to = ");
    Serial.println(myAccessPoint);
    Serial.print("<processWiFiEvent> - Robot Network Host Name = ");
    Serial.println(myHostName);
    Serial.print("<processWiFiEvent> - Robot IP Address = ");
    Serial.println(myIPAddress);
    Serial.print("<processWiFiEvent> - Robot MAC Address = ");
    Serial.println(myMACaddress);
    wifi_connected = true;
    AMDP_PRINTLN("<processWiFiEvent> Use MAC address to create MQTT topic trees...");
    cmdTopicMQTT = myHostName + MQTT_IN_CMD;   // Define variable with the full name of the incoming command topic
    balTopicMQTT = myHostName + MQTT_TEL_BAL;  // Define variabe with the full name of the outgoing balance telemetry topic
    metTopicMQTT = myHostName + MQTT_METADATA; // Define variable with full name of the outgoiong metadata topic
    AMDP_PRINT("<processWiFiEvent> cmdTopicMQTT = ");
    AMDP_PRINTLN(cmdTopicMQTT);
    AMDP_PRINT("<processWiFiEvent> balTopicMQTT = ");
    AMDP_PRINTLN(balTopicMQTT);
    AMDP_PRINT("<processWiFiEvent> metTopicMQTT = ");
    AMDP_PRINTLN(metTopicMQTT);
    connectToMqtt();
    break;
  } //case
  default:
  {
    AMDP_PRINT("<processWiFiEvent> Detected unmanaged WiFi event ");
    AMDP_PRINTLN(event);
  } //default
  } //switch
} // processWifiEvent

/**
 * @brief Handle CONNACK from the MQTT broker
 * @param sessionPresent persistent session available flag contained in CONNACK message from MQTT broker.
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
 * |:-----------:|:------------------------------------------------------|
 * |  0  | Connection accepted |
 * |  1  | Connection refused, unacceptable protocol version |
 * |  2  | Connection refused, identifier rejected |
 * |  3  | Connection refused, server unavailable |
 * |  4  | Connection refused, bad user name or password |
 * |  5  | Connection refused, not authorized |
=================================================================================================== */
void onMqttConnect(bool sessionPresent)
{
  runbit(6) ;
  AMDP_PRINTLN("<onMqttConnect> Connected to MQTT");
  AMDP_PRINT("<onMqttConnect> Session present: ");
  AMDP_PRINTLN(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe((char *)cmdTopicMQTT.c_str(), QOS1); // QOS can be 0,1 or 2. We are using 1
  Serial.print("<onMqttConnect> Subscribing to ");
  Serial.print(cmdTopicMQTT);
  Serial.print(" at a QOS of 1 with a packetId of ");
  Serial.println(packetIdSub);
} //onMqttConnect()

/**
 * @brief Handle disconnecting from an MQTT broker
 * @param reason Reason for disconnect
=================================================================================================== */
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  runbit(7) ;
  AMDP_PRINTLN("<onMqttDisconnect> Disconnected from MQTT");
  metadata.mqttDropCnt++; // Increment the counter for the number of MQTT connection drops
  metadata.mqttConAttemptsCnt++;
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0); // Activate mqtt timer (which only runs 1 time)
  }                                     //if
  metadata.mqttConAttemptsCnt = 0;      // Reset the number of attempts made to connect to the MQTT broker
} //onMqttDisconnect()

/**
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
 * |:-----------:|:----------------------------------|
 * |  0  |  Success - Maximum QoS 0 |
 * |  1  |  Success - Maximum QoS 1 |
 * |  2  |  Success - Maximum QoS 2 |
 * | 128 |  Failure |
=================================================================================================== */
void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  runbit(8) ;
  AMDP_PRINTLN("<onMqttSubscribe> Subscribe acknowledged by broker.");
  AMDP_PRINT("<onMqttSubscribe>  PacketId: ");
  AMDP_PRINTLN(packetId);
  AMDP_PRINT("<onMqttSubscribe>  QOS: ");
  AMDP_PRINTLN(qos);
} //onMqttSubscribe

/**
 * @brief Handle UNSUBACK message from MQTT broker
 * @param packetId Unique identifier of the message. This is the same packet identifier that is in the UNSUBSCRIBE message.
 * # UNSUBACK Message
 * To confirm the unsubscribe, the broker sends an UNSUBACK acknowledgement message to the client. This message contains only the 
 * packet identifier of the original UNSUBSCRIBE message (to clearly identify the message). After receiving the UNSUBACK from the 
 * broker, the client can assume that the subscriptions in the UNSUBSCRIBE message are deleted.
=================================================================================================== */
void onMqttUnsubscribe(uint16_t packetId)
{
  runbit(9) ;
  AMDP_PRINTLN("Unsubscribe acknowledged.");
  AMDP_PRINT("  packetId: ");
  AMDP_PRINTLN(packetId);
} //onMqttUnsubscribe()

/**
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
 * 
 * # Commands
 * When WiFi is available and  there is an MQTT broker availabe, TWIPe is always subscibed to the topic {robot name}/commands. All
 * incoming messages from that topic are checked against a known list of commands and get processed accordingly. Commands that are
 * not recognized get logged and ignored.
 * 
 * ## Table of Known Commands
 * | Command                | Description                                                                                            |
 * |:-----------------------|:-----------------------------------------------------------------------------------------------|
 * | balTelON               | Causes balance telemetry data to be output |  
 * | balTelOFF              | Causes balance telemetry data to stop being output |  
 * | balTelCON              | Causes balance telemetry to be published to the local console |  
 * | balTelMQTT             | Causes balance telemetry to be published to the MQTT broker topic {robot name}/metadata |  
 * | metadataON             | Causes metadata to be published to the MQTT broker topic {robot name}/metadata |  
 * | metadataOFF            | Causes metadata to stop being published to the MQTT broker |  
 * | metadataCON            | Causes metadata to be published to the local console |
 * | metadataMQTT           | Causes metadata to be published to the MQTT broker topic {robot name}/telemetry/balance | 
 * | pid_p_gain=<float>     | sets the gain for the P part of PID balancing to the specified floating point number
 * | pid_i_gain=<float>     | sets the gain for the I part of PID balancing to the specified floating point number
 * | pid_d_gain=<float>     | sets the gain for the D part of PID balancing to the specified floating point number
 * | smoother=<float>       | sets the control value for the motor speed smoothing method to <float>. zero disables smoothing
 * | target_angle=<float>   | sets the target angle in degrees to <float>. Usually within a few degrees of zero
 * | motor_int_slow=<Int>   | sets bot_slow to the given integer. Bigger number is lower bottom speed
 * | motor_int_fast=<Int>   | sets bot_fast to the given integer. Smaller number is faster top speed
 * | active_angle=<float>   | sets angle when bot enters bs_active and starts balancing. usually around 1 - 2 degrees
=================================================================================================== */
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  runbit(10) ;
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
  String tmp = String(payload).substring(0, len);
  AMDP_PRINT("<onMqttMessage> Message to process = ");
  AMDP_PRINTLN(tmp);
  if (tmp == "balTelCON")
  {
    AMDP_PRINTLN("<onMqttMessage> Publish telemetry data to console");
    baltelMsg.destination = TARGET_CONSOLE;
  } //if
  else if (tmp == "balTelMQTT")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing telemetry data to MQTT broker");
    baltelMsg.destination = TARGET_MQTT;
  } //elseif
  else if (tmp == "balTelON")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of telemetry data now ON");
    baltelMsg.active = true;
  } //elseif
  else if (tmp == "balTelOFF")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of telemetry data now OFF");
    baltelMsg.active = false;
  } //elseif
  else if (tmp == "metadataCON")
  {
    AMDP_PRINTLN("<onMqttMessage> Publish metadata to console");
    metadataMsg.destination = TARGET_CONSOLE;
  } //elseif
  else if (tmp == "metadataMQTT")
  {
    AMDP_PRINTLN("<onMqttMessage> Publish metadata to MQTT broker");
    metadataMsg.destination = TARGET_MQTT;
  } //elseif
  else if (tmp == "metadataON")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of metadata now ON");
    metadataMsg.active = true;
  } //elseif
  else if (tmp == "metadataOFF")
  {
    AMDP_PRINTLN("<onMqttMessage> Publishing of metadata now OFF");
    metadataMsg.active = false;
  } //elseif
  else if(tmp.substring(0,10) == "pid_p_gain")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    pid_p_gain = firstValue.toFloat();
    AMDP_PRINT("<onMqttMessage> Set pid_p_gain to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
  else if(tmp.substring(0,10) == "pid_i_gain")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    pid_i_gain = firstValue.toFloat();
    AMDP_PRINT("<onMqttMessage> Set pid_i_gain to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
  else if(tmp.substring(0,10) == "pid_d_gain")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    pid_d_gain = firstValue.toFloat();
    AMDP_PRINT("<onMqttMessage> Set pid_d_gain to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
  else if(tmp.substring(0,8) == "smoother")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    smoother = firstValue.toFloat();
    AMDP_PRINT("<onMqttMessage> Set smoother to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
   else if(tmp.substring(0,12) == "target_angle")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    robotState.targetAngleDegrees = firstValue.toFloat();
    AMDP_PRINT("<onMqttMessage> Set robotState.targetAngleDegrees to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
   else if(tmp.substring(0,14) == "motor_int_slow")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    bot_slow = firstValue.toInt();
    AMDP_PRINT("<onMqttMessage> Set bot_slow to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
   else if(tmp.substring(0,14) == "motor_int_fast")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    bot_fast = firstValue.toInt();
    AMDP_PRINT("<onMqttMessage> Set bot_fast to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
   else if(tmp.substring(0,12) == "active_angle")
  {
    int equalIndex = tmp.indexOf('=');
    String firstValue = tmp.substring(equalIndex+1,len);
    Balance.activeAngle = abs(firstValue.toFloat());
    AMDP_PRINT("<onMqttMessage> Set bot_fast to ");
    AMDP_PRINTLN(firstValue);
  } //elseif
  else
  {
    AMDP_PRINTLN("<onMqttMessage> Unknown command. Doing nothing");
    metadata.unknownCmdCnt++; // Increment the counter that tracks how many unknown commands have been received
  }                           //else
} //onMqttMessage()

/**
 * @brief Handle the reciept of a PUBACK message message from MQTT broker
 * @param packetId Unique identifier of the message.
=================================================================================================== */
void onMqttPublish(uint16_t packetId)
{
  runbit(11) ;
  tm_MQpubCnt ++ ;         // count the number of times this routines executes between balance telemetry
//  AMDP_PRINTLN("Publish acknowledged.");
//  AMDP_PRINT("  packetId: ");
//  AMDP_PRINTLN(packetId);
} //onMqttPublish()

/**
 * @brief Manage multiple attempts to connect to the WiFi network  
=================================================================================================== */
void connectToNetwork()
{
  int maxConnectionAttempts = 20; // Maximum number of Access Point connection attemts
  wifiCurrConAttemptsCnt = 0;     // Number of Access Point connection attempts made during current connection/reconnection effort
  String tmpHostNameVar;          // Hold WiFi host name created in this function
  AMDP_PRINT("<connectToNetwork> Try connecting to Access Point ");
  AMDP_PRINTLN(mySSID);
  WiFi.onEvent(WiFiEvent); // Create a WiFi event handler
  connectToWifi();
  delay(1000); // give it some time to establish the connection
  while ((WiFi.status() != WL_CONNECTED) && (maxConnectionAttempts > 0))
  {
    delay(1000); //  wait between reattempts
    AMDP_PRINT("<connectToNetwork> Re-attempting connection to Access Point. Connect attempt count down = ");
    AMDP_PRINTLN(maxConnectionAttempts);
    AMDP_PRINT("<connectToNetwork>  current Wifi.status() is: ");
    AMDP_PRINTLN(wifiSt[WiFi.status()]);
    int WFs = WiFi.status(); // keep it stable during following tests
    if (WFs == 1 || WFs == 4 || WFs == 5 || WFs == 6 || WFs == 0)
    {
      connectToWifi(); // things went bad enough to need another connect attempt
      delay(1500);     // give it some time to make connection
    }                  //if
    maxConnectionAttempts--;
    wifiCurrConAttemptsCnt++;
  } //while
  if (WiFi.status() != WL_CONNECTED)
  {
    AMDP_PRINTLN("<connectToNetwork> Connection to network FAILED");
  } //if
  else
  {
    AMDP_PRINTLN("<connectToNetwork> Connection to network SUCCEEDED");
  } //else
} //connectToNetwork()

/**
 * @brief This function translates the type of encryption that an Access Point (AP) advertises (an an ENUM) 
 * and returns a more human readable description of what that encryption method is.
 */
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

/**
 * @brief This function scans the WiFi spectrum looking for Access Points (AP). It selects the AP with the 
 * strongest signal which is included in the known network list.
=================================================================================================== */
void scanNetworks()
{
  int numberOfNetworks = WiFi.scanNetworks(); // Used to track how many APs are detected by the scan
  int StrongestSignal = -127;                 // Used to find the strongest signal. Set as low as possible to start
  int SSIDIndex = 0;                          // Contains the SSID index number from the known list of APs
  bool APknown;                               // Flag to indicate if the current AP appears in the known AP list
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
    for (int j = 0; j < numKnownAPs; j++)
    {
      // If the current scanned AP appears in the known AP list note the index value and flag found
      if (WiFi.SSID(i) == SSID[j])
      {
        APknown = true;
        SSIDIndex = j;
        AMDP_PRINTLN("<scanNetworks> This is a known network");
      } //if
    }   //for

    // If the current AP is known and has a stronger signal than the others that have been checked
    // then store it in the variables that will be used to connect to the AP later
    if ((APknown == true) && (WiFi.SSID(i).toInt() > StrongestSignal))
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

/**
 * @brief Print leading zeros for a binary number
 * @note Code taken from Peter H Anderson example
 * @arg int8_t v = number to dispay
 * @arg int8_t num_places = number of bits to display (normally a multile of 8)
=================================================================================================== */
void printBinary(byte v, int8_t num_places)
{
  int8_t mask = 0, n;
  for (n = 1; n <= num_places; n++)
  {
    mask = (mask << 1) | 0x0001;
  }             //for
  v = v & mask; // truncate v to specified number of places
  while (num_places)
  {
    if (v & (0x0001 << (num_places - 1)))
    {
      Serial.print("1");
    } //if
    else
    {
      Serial.print("0");
    } //else
    --num_places;
    if (((num_places % 4) == 0) && (num_places != 0))
    {
      Serial.print("_");
    } //if
  }   //while
} //printBinary()

/**
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
=================================================================================================== */
void publishMQTT(String topic, String msg)
{
  runbit(12) ;
  char tmp[NUMBER_OF_MILLI_DIGITS];
  itoa(millis(), tmp, NUMBER_OF_MILLI_DIGITS);
  String message = String(tmp) + "," + msg;
  if (topic == "balance")
  {
    uint16_t packetIdPub1 = mqttClient.publish((char *)balTopicMQTT.c_str(), QOS1, false, (char *)message.c_str()); // QOS 0-2, retain t/f
    AMDP_PRINT("<publishMQTT> PacketID for publish to balance topic is ");
    AMDP_PRINTLN(packetIdPub1);
  } //if
  else if (topic == "metadata")
  {
    uint16_t packetIdPub1 = mqttClient.publish((char *)metTopicMQTT.c_str(), QOS1, false, (char *)message.c_str()); // QOS 0-2, retain t/f
    AMDP_PRINT("<publishMQTT> PacketID for publish to metadata topic is ");
    AMDP_PRINTLN(packetIdPub1);
  } //else if
  else
  {
    AMDP_PRINT("<publishMQTT> ERROR. Unknown MQTT topic tree - ");
    AMDP_PRINTLN(topic);
  } //else
} //publishMQTT()

/**
 * @brief Control stepping of both stepper motors. 
 * @note ISR for each motor calls this routine with the motor number index.  
 * @param index Which motor the interrupt is for. 0 = right motor, 1 = left motor
 * @param mod Odometer modifier. Handle updating the trip odometer for both polarities (directions)
=================================================================================================== */
void stepMotor(int index, uint mod)
{
runbit(13) ;  
  uint8_t gpioPin[2]; // Create 2 element array to hold values of the left and right motor pins
  gpioPin[RIGHT_MOTOR] = gp_DRV1_STEP; // Element 0 holds right motor GPIO pin value
  gpioPin[LEFT_MOTOR] = gp_DRV2_STEP; // Element 1 holds left motor GPIO pin value
  if (stepperMotor[index].interruptCounter == 1) // If this is the rising edge of the step signal
  {
    digitalWrite(gpioPin[index], HIGH);
  }                                              //if
  if (stepperMotor[index].interruptCounter == 2) // If this is the falling edge of the step signal
  {
    digitalWrite(gpioPin[index], LOW);
  }                                                                         //if
  if (stepperMotor[index].interruptCounter >= stepperMotor[index].interval) // If this is the end of the delay period
  {
    //  portENTER_CRITICAL_ISR(&rightMotorTimerMux);
    stepperMotor[index].interruptCounter = 0;
    //  portEXIT_CRITICAL_ISR(&rightMotorTimerMux);
  } //if
  else
  {
    //  portENTER_CRITICAL_ISR(&rightMotorTimerMux);
    stepperMotor[index].interruptCounter++;
    //  portEXIT_CRITICAL_ISR(&rightMotorTimerMux);
  } //else
} //rightMotorTimerISR()

/** 
 * @brief ISR for right stepper motor that drives the robot
 * 
=================================================================================================== */
//TODO Put balance logic in here
void IRAM_ATTR rightMotorTimerISR()
{
  runbit(14) ;
  if(Balance.method == bm_catchup)                // this is the ISR for the catchup method of balancing
  { 
      int motor = RIGHT_MOTOR;
      noInterrupts();
      int tmp = Balance.steps;
      stepperMotor[RIGHT_MOTOR].interval = stepperMotor[RIGHT_MOTOR].minSpeed + stepperMotor[RIGHT_MOTOR].speedRange;
      interrupts();
      // Determine motor direction
      if (tmp > 0)
      {
        digitalWrite(gp_DRV1_DIR, LOW);
      } //if
      else
      {
        digitalWrite(gp_DRV1_DIR, HIGH);
      } //else
      stepMotor(motor, tmp);                    //de  vague memory about problems if ISR's in IRAM call routines out of IRAM?
  } // if(Balance.method == bm_catchup)

  if(Balance.method == bm_angle)                // this is the ISR for the angle method of balancing
  { throttle_Rcounter ++;                       // increment our once per interrupt tick counter
    if(throttle_Rcounter > throttle_Rlimit)     // did that take us to the limit value?
    { throttle_Rcounter = 0;                    // yes, reset the counter, which goes upwards
      throttle_Rlimit = throttle_Rsetting;      // reset upper limit to what the background calculated in balanceByAngle()
      if(throttle_Rlimit< 0)                    // negative throttle means backwards
      { digitalWrite(gp_DRV1_DIR,LOW);          // write zero to direction bit on DRV8825 motor controller
        throttle_Rlimit *= -1;                  // get back to a +ve number for counter comparisons
      }
      else digitalWrite(gp_DRV1_DIR,HIGH);      // if not negative limit value, set wheel direction = forward
    }
    else if(throttle_Rcounter == 1) digitalWrite(gp_DRV1_STEP,HIGH);  // start the step pulse at end of first counted tick
    else if(throttle_Rcounter == 2) digitalWrite(gp_DRV1_STEP,LOW);   // end the step pulse at end of second counted tick  
  } // if(Balance.method == bm_angle)



} //rightMotorTimerISR()

/** 
 * @brief ISR for left stepper m otor that drives the robot
 * 
=================================================================================================== */
//TODO Put balance logic in here
void IRAM_ATTR leftMotorTimerISR()
{
  runbit(15) ;
  if(Balance.method == bm_catchup)                // this is the ISR for the catchup method of balancing
  { 
    int motor = LEFT_MOTOR;
    noInterrupts();
    int tmp = Balance.steps;
    stepperMotor[LEFT_MOTOR].interval = stepperMotor[LEFT_MOTOR].minSpeed + stepperMotor[LEFT_MOTOR].speedRange;
    interrupts();
    // Determine motor direction
    if (tmp > 0)
    {
      digitalWrite(gp_DRV2_DIR, LOW);
    } //if
    else
    {
      digitalWrite(gp_DRV2_DIR, HIGH);
    } //else
    stepMotor(motor, tmp); 
  } // if(Balance.method == bm_catchup)

  if(Balance.method == bm_angle)                // this is the ISR for the angle method of balancing
  { throttle_Lcounter ++;                       // increment our once per interrupt tick counter
    if(throttle_Lcounter > throttle_Llimit)     // did that take us to the limit value?
    { throttle_Lcounter = 0;                    // yes, reset the counter, which goes upwards
      throttle_Llimit = throttle_Lsetting;      // reset upper limit to what the background calculated in balanceByAngle()
      if(throttle_Llimit< 0)                    // negative throttle means backwards
      { digitalWrite(gp_DRV2_DIR,LOW);          // write zero to direction bit on DRV8825 motor controller
        throttle_Llimit *= -1;                  // get back to a +ve number for counter comparisons
      }
      else digitalWrite(gp_DRV2_DIR,HIGH);      // if not negative limit value, set wheel direction = forward
    }
    else if(throttle_Lcounter == 1) digitalWrite(gp_DRV2_STEP,HIGH);  // start the step pulse at end of first counted tick
    else if(throttle_Lcounter == 2) digitalWrite(gp_DRV2_STEP,LOW);   // end the step pulse at end of second counted tick  
  } // if(Balance.method == bm_angle)
} //leftMotorTimerISR()

/**
 * @brief Calculate what needs to be done to get the robot's centre of mass (COM) over its drive wheels 
 * @param angleRadians Angle of robot lean in radians. 
 * @note We are working with Yaw/Pitch/Roll data (and only using roll). Other options include euler, 
 * quaternion, raw acceleration, raw gyro, linear acceleration and gravity. 
 * See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
=================================================================================================== */
void calcBalanceParmeters(float angleRadians)
{
  runbit(16) ;
//de suggest removing function argument, and using stored tilt value
//de  disabling interrupts isn't effective because angle updates are done in background, in readIMU  
  noInterrupts();      // Prevent other code from updating balance variables while we are changing them
  Balance.angleRadians = angleRadians;
  //de following line is already done in readIMU, with out of range protection
  Balance.angleDegrees = Balance.angleRadians * 180 / PI;                        // Convert radians to degrees
  //de  does the math for the following assume vertical is 0 degrees, or 90 degrees?
  Balance.centreOfMassError = robot.heightCOM - (robot.heightCOM * sin(Balance.angleRadians)); // Calc distance COM is from 90 degrees
  //de  Balance.COMError = robot.heightCOM * sin(Balance.tilt * DEG_TO_RAD);     // the catch-up distance
  Balance.steps = Balance.centreOfMassError / robot.distancePerStep;             // Calc # of steps needed to cover that distance
  interrupts();                                                                  // Allow other code to update balance variables again
  // Assemble balance telemetry string
  //de would it be better to report angles in degrees or radians to MQTT?
  String tmp = String(angleRadians) + "," + String(Balance.centreOfMassError);
  tmp = tmp + "," + String(Balance.steps) + "," + String(stepperMotor[RIGHT_MOTOR].interval);
  if (baltelMsg.active) // If configured to write balance telemetry data
  {
    if (baltelMsg.destination == TARGET_CONSOLE) // If we are to send this data to the console
    {
      Serial.print("<calcBalanceParmeters> ");
      Serial.println(tmp);
    }    //if
    else // Otherwise assume we are to send the data to the MQTT broker
    {
      publishMQTT("metadata", tmp);
    } //else
  }   //if
} // calcBalanceParmeters()

/**
 * @brief Adjust motor controls to minimize how far we are from vertical, using PID tuning 
 * called from loop()
=================================================================================================== */
void balanceByAngle()
{
  float angleErr = Balance.tilt - robotState.targetAngleDegrees;   // difference between current and desired angles
  pid = pid_p_gain * angleErr;            // apply the multiplier for the P in PID
                                          // ignore the I and D in PID, for now
  if(pid >  400) pid = 400;               // range limit pid
  if(pid < -400) pid = -400;
  if(abs(pid) < 5) pid = 0;               // create a dead band to stop motors when robot is balanced

  if(pid > 0) motorInt = int(    bot_slow - (pid/400)*(bot_slow - bot_fast) ) ;  // motorInt is speed interval in timer ticks
  if(pid < 0) motorInt = int( -1*bot_slow - (pid/400)*(bot_slow - bot_fast) ) ;
  if(pid == 0) motorInt = 0 ;

  // experimental  motor speed change smoothing
  // smoother is a variable, and the control parameter - smoother == 0 means don't smooth
  if(smoother != 0)                       // if smoothing is enabled, do it
  {
    int deltaSpd = smoother * (motorInt-lastSpeed);
    motorInt = lastSpeed + deltaSpd;
  }
  noInterrupts();         // block any motor interrupts while we change control parameters
  //d2  reverse direction of wheel rotation, based on observation of Dougs bot
  throttle_Lsetting = stepperMotor[LEFT_MOTOR].directionMod * motorInt;
  throttle_Rsetting = stepperMotor[RIGHT_MOTOR].directionMod * motorInt;
  if(lastSpeed * motorInt < 0 )     // if old and new signs are different, we've reversed desired directions
  {                                 // and we should abort current step in the wrong direction 
      throttle_Lcounter = 9999 ;    // force counter overflow, and thus reading of the new setting
      throttle_Rcounter = 9999 ;
  }
  interrupts();
  lastSpeed = motorInt;                 // remember last speed for smoothing and quick direction change

  // Assemble balance telemetry string

  char flagsInHex[12];                // buffer space for hex string representing runFlagWord
  itoa(runFlagWord,flagsInHex,16);    // convert flags to hex string
  runFlagWord = 0 ;                   // clear flags ASAP, so new routines are seen

  String tmp = String(tm_IMUdelta) +"," + String(tm_readFIFO) + "," + String(tm_dmpGet) + "," + String(tm_allReadIMU) + "," 
   + String(tm_OldbalByAng) + "," + String(Balance.tilt) + "," + String(pid) + "," + String(motorInt) + "," + flagsInHex
   +","+ String(tm_ROLEDtime) +","+ String(tm_MQpubCnt) +","+ String(tm_uMDtime);

   tm_ROLEDtime = 0;                  // don't leave old time hanging around in case routine doesn't run soon.
   tm_LOLEDtime = 0;
   tm_uMDtime = 0;
   tm_IMUdelta = 0;
   tm_readFIFO = 0;
   tm_dmpGet = 0;
   tm_allReadIMU = 0;
   tm_MQpubCnt = 0;

   if (baltelMsg.active) // If configured to write balance telemetry data
  {
    if (baltelMsg.destination == TARGET_CONSOLE) // If we are to send this data to the console
    {
      Serial.print("<balanceByAngle> ");
      Serial.println(tmp);
    }    //if
    else // Otherwise assume we are to send the data to the MQTT broker
    {
      publishMQTT("balance", tmp);              // publish data point string built above.
    } //else
  }   //if
} // balanceByAngle

/**`
 * @brief Send updated metadata about the running of the code.
 * # Metadata
 * There are a number of data points that the TWIPe code tracks in order to assess how the robot's logic is performing. These data 
 * points can be used to pinpoint the cause of perfomance issues as well as play a key role in debugging. Metadata is always being 
 * issued from the updateMetaData() function. When the metaDataON command is received all metadata is directed to the MQTT 
 * broker. When the command metaDataOFF is received all metadata is directed to the serial port. NOte that all the the items in the 
 * table below are found in the payload of the MQTT message. The payload starts with a time stamp in millis() and then is followed 
 * by each of the items in the table below. Each item is separated by a comma. 
 * ## Table of metadata tracked 
 * | Item                     | Details                                                                                               |
 * |:-------------------------|:------------------------------------------------------------------------------------------------------|
 * | WiFi connection          | Number of times a wifi connection had to be established |
 * | WiFi drop                | Number of times the wifi connection dropped |
 * | MQTT connection          | Number of times an MQTT connection had to be established |
 * | MQTT drop                | Number of times an MQTT connection dropped     
 * | MPU6050 DMP read success | Successful attempts to read from the MPU6050 DMP FIFO buffer  |
 * | MPU6050 DMP read fails   | Failed attempts to read from the MPU6050 DMP FIFO buffer  |
 * | Unknown command          | Number of unrecognized commands have been received. |
 * | Left DRV8825 fault       | Number of fault signals sent by the left DVR8825 stepper motor driver |
 * | Right DRV8825 fault      | Number of fault signals sent by the right DVR8825 stepper motor driver |
=================================================================================================== */
void updateMetaData()
{
  runbit(17) ;
  telMilli5 = millis();             // timestamp to get execution time for telemetry
  if (metadataMsg.active) // If configured to write metadata
  {
    String tmp = String(metadata.wifiConAttemptsCnt)
      + "," + String(metadata.wifiDropCnt)
      + "," + String(metadata.mqttConAttemptsCnt)
      + "," + String(metadata.mqttDropCnt)
      + "," + String(metadata.dmpFifoDataPresentCnt)
      + "," + String(metadata.dmpFifoDataMissingCnt)
      + "," + String(metadata.unknownCmdCnt)
      + "," + String(metadata.leftDRVfault)
      + "," + String(metadata.rightDRVfault);

    if (metadataMsg.destination == TARGET_CONSOLE) // If we are to send this data to the console
    {
      AMDP_PRINT("<updateMetaData> ");
      AMDP_PRINTLN(tmp);
    }    //if
    else // Otherwise assume we are to send the data to the MQTT broker
    {
      publishMQTT("metadata", tmp);
    }                                  //else
  }                                    //if
  goMETADATA = millis() + tmrMETADATA; // Reset SERIAL update target time
  tm_uMDtime = millis() - telMilli5;   // telemetry measure: time spent in updateMetadata()
} // updateMetaData()

/**
 * @brief Update Left OLED display with network info: MAC, IP, AccessPoint, Hostname
 * @param 
 * @note This routine is called at end of setup() and from updateRightOLED()   
=================================================================================================== */
void updateLeftOLEDNetInfo()
{
  if(OLED_enable)
  {
    runbit(18) ;
    leftOLED.clear();
    leftOLED.drawString(0, 0, String(myIPAddress));
    leftOLED.drawString(0, 16, String(myMACaddress));
    leftOLED.drawString(0, 32, String(myAccessPoint));
    leftOLED.drawString(0, 48, String(myHostName));
    leftOLED.display();
  }
} //UpdateLeftOLEDNetInfo()

/**
 * @brief Update OLED dipsplay 
 * @param angle Angle of robot lean in radians. To convert to degrees use this formula: degrees = angle * 180 / PI
 * @note We are working with Yaw/Pitch/Roll data (and only using roll). Other options include euler, quaternion, raw acceleration, 
 * raw gyro, linear acceleration and gravity. See MPU6050_6Axis_MotionApps_V6_12.h for more details.   
=================================================================================================== */
void updateRightOLED()
{
  if(OLED_enable)
  {
    telMilli4 = millis();         // timestamp for start of this routine
    runbit(19) ;
    rightOLED.clear();
    rightOLED.drawString(40, 0, String(Balance.tilt));
//    rightOLED.drawString(0, 16, String("Mtr: ")+String(motorInt));    // take this out to see impact on goIMU timing
//    rightOLED.drawString(0, 32, String("PID: ")+String(pid));
    rightOLED.display();
    telMilli5 = millis();                  // timestamp between L & R OLED updates
    tm_ROLEDtime = telMilli5 - telMilli4; // telemetry measure - time spent in updateRightOLED
//  following routine is called once, from setup(), since the info doesn't change!
//    updateLeftOLEDNetInfo();              // put MAC, IP, AccessPoint and Hostname onto left OLED
    tm_LOLEDtime = millis() - telMilli5;  // telemetry measure - time spent in updateLeftOLED
    goOLED = millis() + tmrOLED;          // Reset OLED update target time
  }
} //UpdateRightOLED()

/**
 * @brief Update left OLED dipsplay with current routine being executed within setup()
 * @param stage string indicating the stage within setup() we are about to start
 * @note This routine is only called during the initial execution of setup()   
=================================================================================================== */
void updateLeftOLED(String stage)
{
  runbit(20) ;
  leftOLED.clear();
  leftOLED.drawString(0, 0, String("   Setup() Stage: "));
  leftOLED.drawString(0, 32, String("> ")+String(stage));
 // rightOLED.drawString(0, 32, String("PID: ")+String(pid));
  leftOLED.display();
} //UpdateLeftOLED()

/**
 * @brief Set up a WiFi connection
=================================================================================================== */
void setupWiFi()
{
  runbit(21) ;
  scanNetworks();
  connectToNetwork();
} // setupWiFi()

/**
 * @brief Function called when a message appears on the command topic subsription 
 * @param topic 
 * @param message 
=================================================================================================== */
void subscribed_callback(char *data, uint16_t len)
{
  runbit(22) ;
  // Print out topic name and message
  AMDP_PRINT("<subscribed_callback> Got this command: ");
  AMDP_PRINTLN(data);
} //subscribed_callback

/**
 * @brief Set up communication with an MQTT broker
 * Refer to: https://learn.adafruit.com/introducing-the-adafruit-wiced-feather-wifi/adafruitmqtt
=================================================================================================== */
void setupMQTT()
{
  runbit(23) ;
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);

} //setupMQTT()

/**
 * @brief Set up the LED that is flashed by loop()
=================================================================================================== */
void setupLED()
{
  AMDP_PRINTLN("<setupLED> Enable LED pin");
  pinMode(gp_SWC_LED, OUTPUT); // configure LED for output
} //setupLED()

/**
 * @brief Set up the OLED 
=================================================================================================== */
void setupOLED()
{
  AMDP_PRINTLN("<setupOLED> Initialize L & R OLEDs");
  rightOLED.init();
  rightOLED.setFont(ArialMT_Plain_16);
  rightOLED.setTextAlignment(TEXT_ALIGN_LEFT);
  rightOLED.drawString(32, 20, "My Demo"); //64,22
  rightOLED.display();

  leftOLED.init();
  leftOLED.setFont(ArialMT_Plain_16);
  leftOLED.setTextAlignment(TEXT_ALIGN_LEFT);
  leftOLED.drawString(32, 20, "My Demo"); //64,22
  leftOLED.display();
  AMDP_PRINTLN("<setupOLED> Initialization of L & R OLEDs complete");
} //setupOLED()

/**
 * @brief Set up the MPU6050 using DMP firmware but NO interrupts
=================================================================================================== */
void setupIMU()
{
  // Initialize device
  AMDP_PRINTLN("<setupIMU> Initializing MPU6050...");
  mpu.initialize();
  // Rowbergs latest example sets up DMP interrupt, but doesn't use it. We'll leave interrupt support out.  
  // Verify connection
  AMDP_PRINTLN("<setupIMU> Testing MPU6050 connection...");
  bool tmp = mpu.testConnection();
  if (tmp == true)
  {
    AMDP_PRINTLN("<setupIMU> MPU6050 connection successful");
  } //if
  else
  {
    AMDP_PRINTLN("<setupIMU> MPU6050 connection failed. Halting boot up");
    delay(1000);      // allow serial message to get out before system hangs
    while (1)
      ;
  } //else
  // Load and configure the DMP
  AMDP_PRINTLN(F("<setupIMU> Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(robot.XGyroOffset);
    mpu.setYGyroOffset(robot.YGyroOffset);
    mpu.setZGyroOffset(robot.ZGyroOffset);
    mpu.setXAccelOffset(robot.XAccelOffset);
    mpu.setYAccelOffset(robot.YAccelOffset);
    mpu.setZAccelOffset(robot.ZAccelOffset);
    // Generate offsets and calibrate MPU6050
    // next 2 calls are not in the Rowberg example, but leaving them in for now
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    AMDP_PRINTLN();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    AMDP_PRINTLN("<setupIMU> Enabling DMP...");
    mpu.setDMPEnabled(true);
    AMDP_PRINTLN("<setupIMU> Intentionally NOT enabling DMP interrupts");    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    AMDP_PRINT("<setupIMU> packetSize = ");
    AMDP_PRINTLN(packetSize);
    Balance.method = bm_initialMethod;      // are we balancing by catchup distance, or angle deviation?
  }    //if
  else // If initialization failed
  {
    Serial.print("<setupIMU> DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.print(") = ");
    if (devStatus == 1)
    {
      Serial.println("initial memory load failed");
    } //if
    else if (devStatus == 2)
    {
      Serial.println("DMP configuration updates failed");
    } //if
    else
    {
      Serial.println("cause of failure unknown");
    } //if
    Serial.println("<setupIMU> Boot sequence halted");
    delay(1000);      // allow serial message to get out before system hangs
    // TODO handle improve handling of case where IMU has a startup problem    
    while (1) ;       // loop forever thus halting boot up
  }     //else
} //setupIMU()

/**
 * @brief Set up robot specific configuration based on the ESP32 MAC address
=================================================================================================== */
void cfgByMAC()
{
  tmpHostNameVar = myHostNameSuffix + myMACaddress;
  myMACaddress = formatMAC();
  if (myMACaddress == "BCDDC2F7D6D5") // This is Andrew's bot
  {
    AMDP_PRINTLN("<cfgByMAC> Setting up MAC BCDDC2F7D6D5 configuration - Andrew");
    robot.XGyroOffset = -4691;
    robot.YGyroOffset = 1935;
    robot.ZGyroOffset = 1873;
    robot.XAccelOffset = 16383;
    robot.YAccelOffset = 0;
    robot.ZAccelOffset = 0;
    robot.heightCOM = 5;
    robot.wheelDiameter = 3.937008; // 100mm in inches
    stepperMotor[RIGHT_MOTOR].stepsPerRev = 200;
    stepperMotor[LEFT_MOTOR].stepsPerRev = 200;
    stepperMotor[RIGHT_MOTOR].minSpeed = 300;  // Min effective speed of motor
    stepperMotor[LEFT_MOTOR].speedRange = 300; // Range of speeds motor can effectively use
    stepperMotor[RIGHT_MOTOR].interval = stepperMotor[RIGHT_MOTOR].minSpeed + stepperMotor[RIGHT_MOTOR].speedRange;
    stepperMotor[LEFT_MOTOR].interval = stepperMotor[LEFT_MOTOR].minSpeed + stepperMotor[LEFT_MOTOR].speedRange;
    stepperMotor[RIGHT_MOTOR].directionMod = 1 ;        // rotate as directed by software
    stepperMotor[LEFT_MOTOR].directionMod = 1 ;  
    MQTT_BROKER_IP = "192.168.2.21";
  }                                        //if
  else if (myMACaddress == "B4E62D9EA8F9") // This is Doug's bot
  {
    AMDP_PRINTLN("<cfgByMAC> Setting up MAC B4E62D9EA8F9 configuration - Doug");
    robot.XGyroOffset = 60;
    robot.YGyroOffset = -10;
    robot.ZGyroOffset = -72;
    robot.XAccelOffset = -2070;
    robot.YAccelOffset = -70;
    robot.ZAccelOffset = 1641;
    robot.heightCOM = 5;
    robot.wheelDiameter = 3.937008; // 100mm in inches
    stepperMotor[RIGHT_MOTOR].stepsPerRev = 200;
    stepperMotor[LEFT_MOTOR].stepsPerRev = 200;
    stepperMotor[RIGHT_MOTOR].minSpeed = 300;  // Min effective speed of motor
    stepperMotor[LEFT_MOTOR].speedRange = 300; // Range of speeds motor can effectively use
    stepperMotor[RIGHT_MOTOR].interval = stepperMotor[RIGHT_MOTOR].minSpeed + stepperMotor[RIGHT_MOTOR].speedRange;
    stepperMotor[LEFT_MOTOR].interval = stepperMotor[LEFT_MOTOR].minSpeed + stepperMotor[LEFT_MOTOR].speedRange;
    stepperMotor[RIGHT_MOTOR].directionMod = -1 ;        // rotate as directed by software, then reversed
    stepperMotor[LEFT_MOTOR].directionMod = -1 ;  
    MQTT_BROKER_IP = "192.168.0.99";
  } //else if
  else
  {
    Serial.println("<cfgByMAC> MAC not recognized. Setting up generic configuration");
    robot.XGyroOffset = 135;
    robot.YGyroOffset = -9;
    robot.ZGyroOffset = -85;
    robot.XAccelOffset = -3396;
    robot.YAccelOffset = 830;
    robot.ZAccelOffset = 1890;
    robot.heightCOM = 5;
    robot.wheelDiameter = 3.937008; //100mm in inches
    stepperMotor[RIGHT_MOTOR].stepsPerRev = 200;
    stepperMotor[LEFT_MOTOR].stepsPerRev = 200;
    stepperMotor[RIGHT_MOTOR].minSpeed = 300;  // Min effective speed of motor
    stepperMotor[LEFT_MOTOR].speedRange = 300; // Range of speeds motor can effectively use
    stepperMotor[RIGHT_MOTOR].interval = stepperMotor[RIGHT_MOTOR].minSpeed + stepperMotor[RIGHT_MOTOR].speedRange;
    stepperMotor[LEFT_MOTOR].interval = stepperMotor[LEFT_MOTOR].minSpeed + stepperMotor[LEFT_MOTOR].speedRange;
    stepperMotor[RIGHT_MOTOR].directionMod = -1 ;        // rotate as directed by software, then reversed
    stepperMotor[LEFT_MOTOR].directionMod = -1 ;  
    MQTT_BROKER_IP = "unrecognized MAC";
  } //else
  robot.wheelCircumference = robot.wheelDiameter * PI;
  robot.distancePerStep = robot.wheelCircumference / stepperMotor[RIGHT_MOTOR].stepsPerRev;
  AMDP_PRINT("<cfgByMAC> Wheel circumference = ");
  AMDP_PRINTLN(robot.wheelCircumference);
  AMDP_PRINT("<cfgByMAC> Distance per step = ");
  AMDP_PRINTLN(robot.distancePerStep);
  
} //cfgByMAC()

/**
 * @brief Flash AMBER LED on fron panel button
=================================================================================================== */
void updateLED()
{
  runbit(24) ;
  blinkState = !blinkState;
  digitalWrite(gp_SWC_LED, blinkState);
  goLED = millis() + tmrLED; // Reset LED flashing counter
} // updateLED()

/**
 * @brief Retrieve DMP FIFO data
 * @return boolean rCode. True means there is new DMP data. false means that there is not
 */
// TODO learn about the three different dmpGet commands used here. Do we need them all? What do they do? an we call only 1?
boolean readIMU()
{
  boolean rCode = false;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) // Check to see if there is any data in the DMP FIFO buffer.
  { 
    telMilli2 = millis();                      // telemetry timestamp (gives get fifo info execution time))
    tm_readFIFO = telMilli2 - telMilli1;       // telemetry measurement - time to read packet from dmp FIFO
    mpu.dmpGetQuaternion(&q, fifoBuffer);      // Get the latest packet of Quaternion data
    mpu.dmpGetGravity(&gravity, &q);           // Get the latest packet of gravity data, using quaternion data
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get the latest packet of YPR angles, using gravity data
    telMilli3 = millis();                      // telemetry timestamp (gives Rowberg getDmp routines execution time))
    tm_dmpGet = telMilli3 - telMilli2;         // telemetry measurement: time to execute the 3 mpu.dmpGet* routines

    Balance.tilt = ypr[2] * RAD_TO_DEG - 90. ; // get the Roll, relative to original IMU orientation & adjust
    if (Balance.tilt < -180.) Balance.tilt = 90.;     // avoid abrupt change from +90 to -270, when he's past a face plant
    metadata.dmpFifoDataPresentCnt++;          // Track how many times the FIFO pin goes high and the buffer has data in it
    rCode = true;
  }    //if
  else // If sampling rate is reasonable, but no data is available then something weird happened
  {
    metadata.dmpFifoDataMissingCnt++; // Track how many times the FIFO pin goes high but the buffer is empty
  }                                   //else
  // goIMU = millis() + tmrIMU;          // Reset IMU update counter. do this in loop() instead
  return rCode;
} // readIMU()

/**    
 * @brief Create FreeRTOS timers that run callback functions in their own seperate FreeRTOS threads. 
 * @note For details abut FreeRTOS timers see https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html
 * @note Timers are created but are not started at this point. xTimerStart is used later to start them.
=================================================================================================== */
void setupFreeRTOStimers()
{
  runbit(25) ;
  int const wifiTimerPeriod = 2000;                                    // Time in milliseconds between wifi timer events
  int const mqttTimerPeriod = 2000;                                    // Time in milliseconds between mqtt timer events
  mqttReconnectTimer = xTimerCreate("mqttTimer",                       // Human readable name assigned to timer
                                    pdMS_TO_TICKS(mqttTimerPeriod),    // set timer period. pdMS_TO_TICKS() converts milliseconds to ticks
                                    pdFALSE,                           // Set reload to FALSE so this timer becomes dormant after one run
                                    (void *)0,                         // Timer ID. Not used in our callback function
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt)); // Function the timer calls when it expires
  wifiReconnectTimer = xTimerCreate("wifiTimer",                       // Human readable name assigned to timer
                                    pdMS_TO_TICKS(wifiTimerPeriod),    // set timer period. pdMS_TO_TICKS() converts milliseconds to ticks
                                    pdFALSE,                           // Set reload to FALSE so this timer becomes dormant after one run
                                    (void *)0,                         // Timer ID. Not used in our callback function
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi)); // Function the timer calls when it expires
  if (mqttReconnectTimer == NULL)                                      // Check result of xTimerCreate for mqtt timer
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
=================================================================================================== */
void setupDriverMotors()
{
  runbit(26) ;
  // Set up GPIO pins for the robot's right motor
  AMDP_PRINTLN("<setupDriverMotors> Initialize GPIO pins for right motor");
  pinMode(gp_DRV1_DIR, OUTPUT);   // Set left direction pin as output
  pinMode(gp_DRV1_STEP, OUTPUT);  // Set left step pin as output
  pinMode(gp_DRV1_ENA, OUTPUT);   // Set left enable pin as output
  pinMode(gp_DRV1_FAULT, INPUT);  // Set left driver fault pin as input
  digitalWrite(gp_DRV1_DIR, LOW); // Set left motor direction as forward
  digitalWrite(gp_DRV1_ENA, LOW); // Enable right motor
  // Set up GPIO pins for the robot's left motor
  AMDP_PRINTLN("<setupDriverMotors> Initialize GPIO pins for left motor");
  pinMode(gp_DRV2_DIR, OUTPUT);   // Set right direction pin as output
  pinMode(gp_DRV2_STEP, OUTPUT);  // Set right step pin as output
  pinMode(gp_DRV2_ENA, OUTPUT);   // Set right enable pin as output
  pinMode(gp_DRV2_FAULT, INPUT);  // Set right driver fault pin as input
  digitalWrite(gp_DRV2_DIR, LOW); // Set right motor direction as forward
  digitalWrite(gp_DRV2_ENA, LOW); // Enable left motor
  // Set up right motor driver ISR
  AMDP_PRINTLN("<setupDriverMotors> Configure timer0 to control the right motor");
  uint8_t timerNumber = 0;                                               // Timer0 will be used to control the right motor
  uint16_t prescaleDivider = 80;                                         // Timer0 uses presaler (divider) of 80 so interrupts occur at 1us
  bool countUp = true;                                                   // Timer0 will count up not down
  rightMotorTimer = timerBegin(timerNumber, prescaleDivider, countUp);   // Set Timer0 configuration
  bool intOnEdge = true;                                                 // Interrupt on rising edge of Timer0 signal
  timerAttachInterrupt(rightMotorTimer, &rightMotorTimerISR, intOnEdge); // Attach ISR to Timer0
  bool autoReload = true;                                                // Shoud the ISR timer reload after it runs
  timerAlarmWrite(rightMotorTimer, motorISRus, autoReload);              // Set up conditions to call ISR
  timerAlarmEnable(rightMotorTimer);                                     // Enable ISR
  // Set up left motor driver ISR
  AMDP_PRINTLN("<setupDriverMotors> Configure timer1 to control the left motor");
  timerNumber = 1;                                                     // Timer1 will be used to control the right motor
  prescaleDivider = 80;                                                // Timer1 uses a presaler (divider) of 80 so interrupts occur at 1us
  countUp = true;                                                      // Timer1 will count up not down
  leftMotorTimer = timerBegin(timerNumber, prescaleDivider, countUp);  // Set Timer1 configuration
  timerAttachInterrupt(leftMotorTimer, &leftMotorTimerISR, intOnEdge); // Attach ISR to Timer1
  autoReload = true;                                                   // Shoud the ISR timer reload after it runs
  timerAlarmWrite(leftMotorTimer, motorISRus, autoReload);             // Set up conditions to call ISR
  timerAlarmEnable(leftMotorTimer);                                    // Enable ISR
  // Attach interrupts to track DVR8825 faults
  AMDP_PRINTLN("<setupDriverMotors> Monitor left & right DRV8825 drivers for faults");
  attachInterrupt(gp_DRV1_FAULT, rightDRV8825fault, FALLING);
  attachInterrupt(gp_DRV2_FAULT, leftDRV8825fault, FALLING);
} //setupDriverMotors()

/**
 * @brief Enable or disable motor based on robot angle
=================================================================================================== */
void checkBalanceState()
{
  runbit(27) ;
// check to see if there's been a change in the balance state, and if so, handle the transition
  switch (Balance.state)
  {
    case bs_sleep:
    {
      if( abs(Balance.tilt) < Balance.maxAngleMotorActiveDegrees)  // if robot is within 30 degrees of vertical
      {
        if (digitalRead(gp_DRV1_ENA) == HIGH) // If motor is currently turned off
        {
          AMDP_PRINTLN("<checkTiltToActivateMotors> Enable stepper motors");
          digitalWrite(gp_DRV1_ENA, LOW);
          digitalWrite(gp_DRV2_ENA, LOW);
        }  //if
        Balance.state = bs_awake;       // we're now waiting to hit almost vertical before going active
        AMDP_PRINTLN( "<checkBalanceState> entering state bs_awake");
        // update left eye with network info that is now available and static
        updateLeftOLEDNetInfo();        // put IP, MAC, Accesspoint & MQTT hostname into left eye.
      }    // if (abs(Balance.tilt)

      else // otherwise robot has such a big tilt that it should not be trying to balance
      {
        if (digitalRead(gp_DRV1_ENA) == LOW) // If motor is currently turned ON
        {
          AMDP_PRINTLN("<checkTiltToActivateMotors> Disable stepper motors");
          digitalWrite(gp_DRV1_ENA, HIGH);
          digitalWrite(gp_DRV2_ENA, HIGH);
        } //if
        // ... and stay in bs_sleep state
      }   //else
      break;
    } // case bs_sleep

    case bs_awake:
    { if(abs(Balance.tilt) <= Balance.activeAngle)      // are we almost vertical?
      { Balance.state = bs_active;              // yes, so start trying to balance
        AMDP_PRINTLN( "<checkBalanceState> entering state bs_active");
        
        // publish preliminary info into the MQTT balance telemetry log to help with telemetry interpretation before we get busy

        // first, publish the column titles for the control parameters
        publishMQTT("balance","p-gain,i-gain,d-gain,spd-lo,spd-hi,smooth,tmrIMU,trgt.ang,act.ang");

        // then the values for the control parameters
        publishMQTT("balance",String(pid_p_gain) +","+ String(pid_i_gain) +","+ String(pid_d_gain) +","+ String(bot_slow)
         +","+ String(bot_fast) +","+String(smoother) +","+String(tmrIMU) +","+ String(robotState.targetAngleDegrees)
         +","+ String(Balance.activeAngle));

        // then the column titles for the repeated data points that are published every time we read the IMU and do balancing calculations
        publishMQTT("balance", "IMUdelta,readFIFO,dmpGet,AllReadIMU,OldbalByAng,tilt,pid,MotorInt,runflags,R.O.time,MQpubCnt,uMDtime");

        // the actual data points are published in balanceByAngle()
      }
      if(abs(Balance.tilt > Balance.maxAngleMotorActiveDegrees))    // if we're more than 30 degress from vertical...
      { Balance.state = bs_sleep;                                   // fall back to sleep
        AMDP_PRINTLN("<checkBalanceState> falling back to bs_sleep state");
      }
    break;

    } // case bs_awake
    case bs_active:
    { if(abs(Balance.tilt) >= Balance.maxAngleMotorActiveDegrees)       // have we gone more than 30 degrees from vertical?
      { Balance.state = bs_sleep;    // abort balancing efforts, and go back to waiting for less than 30 degrees tilt
        throttle_Lsetting = 0;       // stop the motors
        throttle_Rsetting = 0;
        motorInt = 0;
        AMDP_PRINTLN( "<checkBalanceState> entering state bs_sleep");

      }                              //  sleep state will look after turning off motor enable
    break;
    } // case bs_active
  } //switch(Balance.state)
} //checkBalanceState()

/**
 * @brief Set the robot's objective
 * @param objective String with human readabe objective for the robot to pursue
 * ## Table of robot objectives 
 * | Item                     | Details                                                                                               |
 * |:-------------------------|:------------------------------------------------------------------------------------------------------|
 * | stand                    | Tries to maintain an angle of 90 degrees and a COM distance from target of 0 inches |
=================================================================================================== */
void setRobotObjective(int objective)
{
  runbit(28) ;
  switch (objective)
  {
  case STATE_STAND_GROUND:
  {
    AMDP_PRINTLN("<setRobotObjective> Robot objective now set to STAND");
    robotState.targetDistance = 0;      // Robot will try to keep COM at 0 inches
  //de  next line was: robotState.targetAngleDegrees = 90; // Robot will try to keep angle at 90 degrees (upright)
    robotState.targetAngleDegrees = 0; // Robot will try to keep angle at 0 degrees (upright)
    break;
  } //case
  default:
  {
    AMDP_PRINT("<setRobotObjective> Ignoring unknown robot objective request ");
    AMDP_PRINTLN(objective);
  } //default
  } //switch
} //setRobotObjective()

/** 
 * @brief Standard set up routine for Arduino programs 
=================================================================================================== */
void setup()
{
  Wire.begin(gp_I2C_IMU_SDA, gp_I2C_IMU_SCL, I2C_bus1_speed);
  Serial.begin(115200); // Open a serial connection at 115200bps
  while (!Serial) ;     // Wait for Serial port to be ready
  Serial.println(F("<setup> Start of setup"));
  
  setupOLED();                           // Setup OLED communication early, so we can show setup() stages
   updateLeftOLED("cfgByMAC");           // display setup routine we are about to execute in bot's right eye
  cfgByMAC();                            // Use the devices MAC address to make specific configuration settings
   updateLeftOLED("setRobotObjective");           // display setup routine we are about to execute in bot's right eye
  setRobotObjective(STATE_STAND_GROUND); // Assign robot the goal to stand upright
   updateLeftOLED("setupLED");           // display setup routine we are about to execute in bot's right eye
  setupLED();                            // Set up the LED that the loop() flashes
   updateLeftOLED("setupFreeRTOStimers");           // display setup routine we are about to execute in bot's right eye
  setupFreeRTOStimers();                 //  User timer based FreeRTOS threads to manage a number of asynchronous tasks
   updateLeftOLED("setupMQTT");           // display setup routine we are about to execute in bot's right eye
  setupMQTT();                           // Set up MQTT communication
   updateLeftOLED("setupWiFi");           // display setup routine we are about to execute in bot's right eye
  setupWiFi();                           // Set up WiFi communication
   updateLeftOLED("setupIMU");           // display setup routine we are about to execute in bot's right eye
  setupIMU();                            // Set up IMU communication
   updateLeftOLED("setupDriverMotors");           // display setup routine we are about to execute in bot's right eye
  setupDriverMotors();                   // Set up the Stepper motors used to drive the robot motion
  updateLeftOLEDNetInfo();             // aftersetup's done, show IP, MAC, AccessPoint and Hostname in left OLED
  goOLED = millis() + tmrOLED;         // Reset OLED update counter
  goLED = millis() + tmrLED;           // Reset LED flashing counter
  goIMU = millis() + tmrIMU;           // Reset IMU update counter
  goMETADATA = millis() + tmrMETADATA; // Reset IMU update counter
  cntLoop = 0;                         // Reset counter that tracks how many iterations of loop() have occurred
  updateLeftOLEDNetInfo();             // output network info once since it's stable, not repeatedly
  Serial.println(F("<setup> End of setup"));
} //setup()

/**
 * @brief Standard looping routine for Arduino programs
=================================================================================================== */
void loop()
{
  cntLoop++; // Increment loop() counter

  if (millis() >= goIMU)                  // use "else if" to only allow one routine to run per loop()...
  {                                       // which increases frequency of checks for goIMU readiness
    goIMU = millis() + tmrIMU;            // Reset IMU update counter right away.
    holdMilli1 = telMilli1;               // remember previous startime to calculate delta time for goIMU starts
    telMilli1 = millis();                 // get a timestamp for telemetry data (gives telemetry publish delay)
    tm_IMUdelta = telMilli1 - holdMilli1; // telemetry measurement: elapsed time since last goIMU call.
    boolean rCode = readIMU();            // Read the IMU. Balancing and data printing is handled in here as well
    holdMilli4 = telMilli4;               // save previous timestamp for BalanceByAngle() calculation
    telMilli4 = millis();                 // telemetry timestamp (gives readIMU execution time)
    tm_allReadIMU = telMilli4 - telMilli1; // telemetry measurement: total time for readIMU routine
    if (rCode)                            //de even if we don't read IMU, should still do balancing?
    {
      checkBalanceState();                // handle balance state changes: sleep, awake, active
      if(Balance.state ==bs_active)       // if we're in a state where we can try to balance
      {                                   // then do so, depending on which method we're using
        if(Balance.method == bm_catchup)
        {
          //de suggest removing the arg in radians, and use stored Balance.tilt value in degrees
          calcBalanceParmeters(ypr[2]);   // Do balancing calculations based on catch up distance
        } // if(Balance.method)
        if(Balance.method == bm_angle)
        { balanceByAngle();               // Do balancing calc's based on angle displacement from vertical, 
                                          // and publish telemetry, resetting runFlagword
        telMilli5 = millis();             // telemetry timestamp (gives balanceByAngle execution time)
        tm_OldbalByAng = telMilli5 - telMilli4; // telemetry measurement: time in BalanceByAngle, reported in NEXT MQTT publish
        }
      } // if(Balance.state...)
    }                               // if rCode
  }                                 // if millis() > goIMU
  else 
  {  if (WifiLastEvent != -1) {processWifiEvent(); }               // if there's a pending Wifi event, handle it
     else
     {  if (millis() >= goOLED) {updateRightOLED(); }              // replace contents of both OLED displays
        else 
        {  if (millis() >= goLED) {updateLED(); }                  // Update the OLED with data
           else 
           {  if (millis() >= goMETADATA) {updateMetaData(); }     // Send data to serial terminal
           }
        }
     }
  }
} //loop()