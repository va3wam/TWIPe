/*************************************************************************************************************************************
 * @file main.cpp
 * @author va3wam
 * @brief TWIPe robot firmware 
 * @details Handles balancing the robot by reading MPU6050 IMU data, using PID to control a pair of open loop bi-polar stepper motors 
 *          via DRV8826 motor drivers, and communicating data and receiving commands through the WiFi interface using the MQTT 
 *          protocol via an MQTT broker running on a raspberry Pi 
 * @version 0.0.22
 * @date 2020-04-25
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * YYYY-MM-DD Description
 * ---------- ----------------------------------------------------------------------------------------------------------------
 * 
 * 2021-02-24 DE: - create wifiDelay for Wifi startup timeout, and extend it to 3000 msec
 *                - simulate BalTelMQTT command in cfyByMac()
 *                - correct calculate of the D part of PID
 *                - fix the selectiveIsum code - had it backwards
 *                - redid IMU calibration, for both bots, and updated cfgByMac()
 * 2021-02-03 DE: - update default params for Andrew and Doug's bots in cfgByMac
 * 2021-01-01 DE: - re-enable display of DRV fault counts on right LED
 * 2021-01-22 DE: - make the displayable balance.ISum be the average, not the total Isum (no change to algorithm)
 *                - if selectiveISum is defined, add the I part to the total PID only if it pushes bot towards vertical, not away from it
 *                - fix bug calculating balance.pidISum at beginning: calculate and use numToSum rather than balance.pidICount
 * 2021-01-19 DE: - switch to QOS = 1 after seeing telemetry anomalies
 * 2021-01-18 DE: - make gpio changes for h/w mod to avoid ESP32 LED connection permanent
 * 2021-01-16 DE: - trivial change (only this line) to test git merge script
 * 2021-01-15 DE: - fix StringToUpper(), changing it from void to returning a string
 *                - change fault count display separator to * so it's not confused with a 1
 *                - add control variable avoidEspLed which changes gpio definitions to correspond to board modification
 *                - that puts DRV1 fault line onto cpu pin 5, gpio 26 rather than gpio 13, which is used by onboard LED
 * 2021-01-14 DE: - redoing commit + push which didn't seem to work
 * 2021-01-13 DE: - recover from mistakenly editing master instead of a branch:
 *                - track time spent in MQTT routines, although it's embedded on other numbers (turns out it's tiny)
 *                - include time for CPU usage calculation in loop's CPU bucket 
 *                - remove unused code planned for loop() optimization
 *                - reformatted entire main.cpp for consistent 3 character indentation
 *                - simplify call to mqttClient.publish() in publishMQTT() to avoid compiler errors. The workaround for the
 *                   compiler error was causing "1" to be printed in serial monitor 
 *                - made MQTT commands and parameters case insensitive, so you can use camel case,
 *                   lower case, upper case, or random case
 *                - add display of left and right DRV fault counters to bottom right corner of right eye. Seeing numbers for
 *                  right eye, even in simple hand held operation.
 * 2020-12-27 DE: - have CPU utilization showing in right eye.
 *                - adjusted a bunch of indentaton - hope it works in ANdrews environment. may depend on tabs = 3 spaces
 * 2020-12-25 DE: - ad CPU usage measurement in variables starting with cu_
 * 2020-12-17 DE: - set motor control values right in motor command handler
 * 2020-12-16 DE: - avoid calling checkBalanceState if we're in motor test mode
 *                - remove display of "My demo" in left OLED, making room for SETUP stage display
 *                - remove tracking of motor enable state - red herring
 *                - un-comment the OLED clear in updateLeftOLED. Misunderstood how overwrite would work.
 *                - update network info in left OLED every time the orange LED is blinked
 * 2020-12-15 DE: -disable motor controllers in setup
 *                -track motor enable state in balance.motorEna  , and avoid unnecessary acess to ENA bit
 * 2020-12-13 DE: -undid change to Doug's wheel direction factor.
 * 2020-11-26 DE: -change Doug's wheel direction factor, & other defaults due to new stepper motors
 * 2020-11-11 DE: -default IP adress string to "-no IP address-"
 *                - generalize updateLeftOLED() to take 2 strings that overlay lines 2 and 3 on OLED
 * 2020-10-27 DE: -expand possible PID error history to 200 past values, still controlled by balance.pidICount
 *                -added Evt (event) MQTT dataflow, and hthEvt MQTT topic to allow bot to report asynchronous events,
 *                 using publishEvent() routine and add a test event just before spreadsheet titles come out
 *                -make the tmrIMU parameter a controllable run time variable rather than a compile time constant, and report
 *                 it in MQTT getBalVars command output
 * 2020-10-16 DE: null commit to flush changes to origin
 * 2020-10-16 DE: PID calcs: use average of historical values for I, rather than sum, and use targetAngle in all vertical checks
 * 2020-10-07 DE: change wheel speed control so pid is proportionally mapped to ground speed, rather than motorInt              motorInt that's proportional.
 * 2020-09-30 DE: Milestone: balanced reliably for minutes on carpet with smaller PID parameters
 *                PID gains (5,0,0), slow,fast(800,300), smother=0, target angle (per bot), active angle =1
 *                -fix bug where active angle doesn't consider target angle
 * 2020-09-23 DE: -discard changes since last merge, after backing up main and topics files
 *                -define new MQTT topics as MQTTTop_<text string of topic>
 *                -restructure publishMQTT()
 *                -add documentation of telemetry balance fields in balanceByAngle
 * 2020-08-13 DE: -correct dte in previous version description
 *                -move the new MQTT commands into background, dispatched from loop() if command arrival timestamp is non-zero
 * 2020-08-13 DE: -add MQTT commands getbalvar, gethealthvar and gethealthtel with command arrival timestamps. Need to move them from
 *                 interrupting level to background (git merged)
 * 2020-07-22 DE: -slow down tmrIMU from 6 to 11 milliseconds, so IMU's dmp has enough time to provide new data.
 *                -revert to QOS 0
 *                - disable debug mode, eliminating serial I/O and interrupts, to reduce overhead & jitter
 * 2020-07-10 DE: -re-structure the structures, merging in most of the isolated by related values
 *                -make use of single motor ISR unconditional, optimizing out if statements
 *                -rename robot struct to attributes, & put non-changing attributes in it, including non-duplicated wheel stuff
 *                -rename metadata struct to health, and metadataMsg to healthMsg
 *                -create left and right structs, which contain wheel-specific balancing params, for now
 *                -remove baltel* and metadata* MQTT commands and replace with new pseudo variables for setvar command:
 *                      balTelOFF, balTelCON, balTelMQTT, healthMsgOFF, healthMsgCON and healthMsgMQTT
 * 2020-07-13     -added AMDP_PRINT2() and AMDP_PRINT2LN()  to declutter debug output
 *                -made MQTT QOS a compile time parameter, controlled by MQTTQos
 *                -implement MQTT command getvars which writes control variables to MQTT, and remove param dump from setvar
 *  * to do: make an MPU struct. already taken, use IMU instead.
 * 0.0.23  2020-07-01 DE: -remove display of tilt angle in right OLED to save compute cycles
 *                        -crank tmrIMU down to 6 msec
 *                        -allow a single ISR for motor controllers, compile time control: bool single_drv_isr
 *                        -first attempt at I part of PID, making pid_i_gain parameter active
 *                        -adjusting the deadband size in balanceByAngle upwards from original setting of 5 (later undone)
 *                        -moved the publishing of telemetry titles to when we enter the awake balancing state, avoiding balancing impact
 * 0.0.22  2020-06-24 DE: -add memory of recent angle errors for I part of PID - incomplete.
 *                        -allow motor testing without balancing, using negative values for bot_slow and bot_high
 *                          if bot_slow is -ve, it's the desired MotorInt value for  left wheel, including the -ve sign
 *                          if bot_fast is -ve, it's the desired MotorInt value for right wheel, including the -ve sign
 *                          if bot_slow is -ve and bot_fast = 0, then bot_slow's speed will be used for both wheels
 *                         -play with target angle = 1, and tmrIMU=10
 * 0.0.21  2020-06-21 AM: Added setvar command to allow for select variables to be set remotely to speed up balance tuning
 * 0.0.20  2020-06-19 AM: Moved control parameter MQTT messages to their own MQTT topic. Also moved headings for balance and  control
 *                        parameters to their own MQTT topic. Ths will make things friendly to MQTT remote clients while maintaining
 *                        the spreadsheet process we are already using. Also corrected brief text for this code to more accurately 
 *                        reflect what this code is along with a details tag that offers greatewer details on how this code works  
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
 *                     - remove balance.state from balance telemetry - not useful
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
   // #include <Fonts/FreeMono12pt7b.h>         // http://oleddisplay.squix.ch/#/home  (monospaced, lato, sanserif)
   // #include <Fonts/FreeMono9pt7b.h>          // saw somewhere that 9 and 12 point fonts are recommended for .96 inch
#include <MPU6050-fix2764.h>                        // for MPU6050
// require the version that has fix for misplaced parenthesis at line 2764
#include <huzzah32_pins.h>                          // Defines our usage of the GPIO pins for Adafruit Huzzah32 dev board
// our own creation
#include <i2c_metadata.h>                           // Defines all I2C related info including device addresses, bus pins and bus speeds
// our own creation
#include <known_networks.h>                         // Defines Access points and passwords that the robot can scan for and connect to
// our own creation
#include <AsyncMqttClient.h> // for Message Queuing Telemetry Support
// from https://github.com/marvinroger/async-mqtt-clientFupOLED()

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
   // print 2 things, like a label followed by a variable: AMDP_PRINT2(" Wifi return code: ", Wifi.returnCode)
   #define AMDP_PRINT2(x,y) Serial.print(x); Serial.print(y)
   #define AMDP_PRINT2LN(x,y) Serial.print(x); Serial.println(y)

#else // Map macros to "do nothing" commands so that when is not TRUE these commands do nothing
   #define AMDP_PRINT(x)
   #define AMDP_PRINTLN(x)
   #define AMDP_PRINTF(x, y)
   #define AMDP_PRINT2(x,y) 
   #define AMDP_PRINT2LN(x,y) 
#endif

// compile time feature controls ////////////////////////////////////////////////////////////////////////////////

// Define which core the Arduino environment is running on
#if CONFIG_FREERTOS_UNICORE    // If this is an SOC with only 1 core
#define ARDUINO_RUNNING_CORE 0 // Arduino is running on that one core
#else                          // If this is an SOC with more than one core (2 is the ony other option at ths point)
#define ARDUINO_RUNNING_CORE 1 // Arduino is running on the second core
#endif

#define MQTTQos 1                     // use Quality of Service level 1 or 0? (0 has less overhead)
bool OLED_enable = true;              // allow disabling OLED for performance troubleshooting
#define selectiveISum true            // only use the I in PID if it pushes us towrds vertical, not away from it
#define wifiDelay 3000                // number of milliseconds to wait between WiFi connect attempts

// struct robotAttributes attribute definition =========================================
typedef struct
{
  float heightCOM = 0;                 // Height from ground to Center Of  Mass of robot in inches
  float wheelDiameter = 3.937008;      // Diameter of wheels in inches. https://www.robotshop.com/en/100mm-diameter-wheel-5mm-hub.html
  float wheelCircumference;            // Diameter x pi
  int stepsPerRev = 200;               // How many steps it takes to do a full 360 degree rotation
  float distancePerStep = 0;           // Distance travelled per step of motor in inches
  int16_t XGyroOffset;                 // Gyroscope x axis (Roll)
  int16_t YGyroOffset;                 // Gyroscope y axis (Pitch)
  int16_t ZGyroOffset;                 // Gyroscope z axis (Yaw)
  int16_t XAccelOffset;                // Accelerometer x axis
  int16_t YAccelOffset;                // Accelerometer y axis
  int16_t ZAccelOffset;                // Accelerometer z axis
} robotAttributes;                     // Structure of attributes of the robot

static volatile robotAttributes attribute; // Object of attributes of the robot

// struct state robotState definition ===========================================================
typedef struct
{
   // possible values for following activity variable
      #define STATE_STAND_GROUND 0
      #define STATE_MOVE_FORWARD 1
      #define STATE_MOVE_BACKWARD 2
      #define STATE_TURN_RIGHT 3
      #define STATE_TURN_LEFT 4
      #define STATE_PARAMETER_UNUSED 0
      #define STATE_TEST_MOTOR 99
   int activity = STATE_STAND_GROUND;      // The current objective that the robot is pursuing
   int parameter = STATE_PARAMETER_UNUSED; // A parameter used by some modes such as turn left and right
   float targetDistance = 0;               // Target distance robot wants to maintain
   float targetAngleDegrees = 0;           // Target angle the robot wants to maintain to achieve the target distance. 0 = stay vertical
   //                                      // note that this is different than balance.targetAngle, for short term balance feedback
} state;                                   // Structure for stepper motors that drive the robot

static volatile state robotState;          // Object of states the robot is in pursuing vaious goals

// Define OLED constants, classes and global variables
SSD1306 rightOLED(rightOLED_I2C_ADD, gp_I2C_LCD_SDA, gp_I2C_LCD_SCL);
SSD1306 leftOLED(leftOLED_I2C_ADD, gp_I2C_LCD_SDA, gp_I2C_LCD_SCL);  // same I2C bus, but different address

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
String myIPAddress = "-no IP address-";     // IP address of the SOC.
String myAccessPoint = "-no access point-"; // WiFi Access Point that we managed to connected to
String myHostName = "-no hostname-";        // Name by which we are known by the Access Point
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
// Note device name is prepended in MQTT publish routine
// Note listens for commands on <device name><commands>

const char* MQTT_BROKER_IP = "not-assigned"; // Need to make this a fixed IP address
/*
MQTT Activities are:
  bal - balancing
  nav - navigation
  hth - health
  cfg - configuration
  sht - spreadsheet support

MQTT dataflows are:
  Tel - telemetry 
  Ctl - reply to request to read control parameters 
  Com - comments used in spread sheet analysis 
  Evt - reporting occurrence of an asynchronous event woth noting

Topics are an activity and a dataflow concatenated

Commands for any activity are sent to the robot using the topic "commands"
*/

#define MQTTTop_balTel "/balTel"                      // outgoing balancing telemetry topic
#define MQTTTop_balCtl "/balCtl"                      // outgoing reply to request to get balancing control params
#define MQTTTop_navTel "/navTel"                      // outgoing navigation telemetry topic
#define MQTTTop_navCtl "/navCtl"                      // outgoing reply to request to get navigation control params
#define MQTTTop_hthTel "/hthTel"                      // outgoing health telemetry topic
#define MQTTTop_hthCtl "/hthCtl"                      // outgoing reply to request to get health control params
#define MQTTTop_cfgCtl "/cfgCtl"                      // outgoing reply to request to get configuration control params
#define MQTTTop_shtCom "/shtCom"                      // outgoing spreadsheet comment topic

#define MQTTTop_commands "/commands"                    // incoming commands from MQTT topic


#define MQTT_BROKER_PORT 1883 // Use 8883 for SSL
#define MQTT_USERNAME "NULL" // Not used at this time. To do: secure MQTT broker
#define MQTT_KEY "NULL" // Not used at this time. To do: secure MQTT broker
/* old topics will be reworked to use new topics above
#define MQTT_IN_CMD "/commands" // Topic branch for incoming remote commands
#define MQTT_COMMENT "/spreadsheet/comment" // Topic tree for outgoing balance telemetry heading data
#define MQTT_BAL_TEL "/balance/telemetry" // Topic tree for outgoing balance telemetry data
#define MQTT_BAL_REPLY "balancing/replyControlParams"   // for responses to MQTT get control params commands
#define MQTT_HEALTH_REPLY "/health/replyControlParams" // Topic tree for outgoing metadata about the robot
// #define MQTT_CNTL_PARM_HEAD "/controlHeadings" // Topic tree for outgoing metadata about the robot
#define MQTT_CNTL_PARM "/controlParameters" // Topic tree for outgoing metadata about the robot
*/
// #define QOS1 1 // Quality of service level 1 ensures at least one copy of messages are delivered
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
String cmdTopicMQTT = "NOTHING"; // Full path to incoming command topic from MQTT broker
String balTopicHeadingMQTT = "NOTHING"; // Full path to outgoing balance heading topic to MQTT broker
String balTopicMQTT = "NOTHING"; // Full path to outgoing balance telemetry topic to MQTT broker
String metTopicMQTT = "NOTHING"; // Full path to outgoing metadata topic to MQTT broker
String cntlParmHeadingMQTT = "NOTHING"; // Full path to outgoing control parameter heading topic to MQTT broker
String cntlParmMQTT = "NOTHING"; // Full path to outgoing control parameter topic to MQTT broker

// Define global motor control variables and structures.
#define motorISRus 20               // Number of microseconds between motor ISR calls
hw_timer_t *motorTimer = NULL;      // Pointer to motor ISR
float motorPrecalc = 0;             // holds precalculation to optimize balanceByAngle's calculation of motorInt

// stepperMotor struct definition, one per wheel.  ==================================================
typedef struct
{
  // interrupt level variables...
  volatile int tickCounter;                    // working counter of 20uS timer ticks in motor controller ISR
  volatile int tickLimit;                      // limit that determines step length in timer ticks
  volatile int tickSetting;                    // value for current limit, as calculated in BalancceByAngle()

} motorControl;                               // Structure for stepper motors that drive the robot
static volatile motorControl left;            // Define a struct for left  wheel, example:  left.tickCounter
static volatile motorControl right;           // Define a struct for right wheel, example: right.tickCounter

// Define global control variables.
#define NUMBER_OF_MILLI_DIGITS 10 // Millis() uses unsigned longs (32 bit). Max value is 10 digits (4294967296ms or 49 days, 17 hours)
// change tmrIMU to be a variable in balance struct so we can change it on the fly
// #define tmrIMU  12                // Milliseconds to wait between reading data to IMU over I2C, and doing balancing calculations
#define tmrOLED 200               // Milliseconds to wait between sending data to OLED over I2C
#define tmrMETADATA 1000          // Milliseconds to wait between sending data to serial port
#define tmrLED 1000 / 2           // Milliseconds to wait between flashes of LED (turn on / off twice in this time)
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
unsigned long telMilli5;          // timestamp used for telemetry reporting

unsigned long tm_IMUdelta;        // telemetry value: measured time between goIMU calls. should be tmrIMU
unsigned long tm_readFIFO;        // telemetry value: how long the mpu.dmpGetCurrentFIFOPacket(fifoBuffer) execution took
unsigned long tm_dmpGet;          // telemetry value: how long the dmpGet* calls after above call took
unsigned long tm_allReadIMU;      // telemetry value: how long the readIMU execution took
unsigned long tm_OldbalByAng;     // telemetry value: how long the PREVIOUS balanceByAngle took
unsigned long tm_ROLEDtime;       // telemetry measure: time spent in right OLED update
unsigned long tm_LOLEDtime;       // telemetry measure: time spent in left OLED update 
unsigned long tm_uMDtime;         // telemetry measure: time spent in updateMetadata()
int tm_MQpubCnt = 0;              // telemetry measure: count of onMQTTpublish() executions

unsigned long runFlagWord;        // telemetry word with bit coded flags indicating if a routine has run since last telemetry
//                                // the indicated routine has runbit(n); at its beginning, where n is it's bit number, 0 - 31
//                                // the runFlagWord is cleared after each balance telemetry publish

// CPU Usage measurement variables
int cu_secStart = 0;              // timestamp for start of current CPU measurement second
int cu_loopStart = 0;             // timestamp for start of current loop() interation
int cu_lastLoopEnd = 0;           // timestamp for end of loop() before OS does it's stuff

// CPU usage measurements, in microseconds
int cu_IMU = 0;                   // time spent in goIMU controlled part of main loop
int cu_wifi = 0;                  // time spent in wifi processing in main loop
int cu_OLED = 0;                  // time spent in goOLED controlled part of main loop
int cu_LED = 0;                   // time spent in goLED controlled part of main loop
int cu_metaData = 0;              // time spent in goMetadata controlled part of main loop
int cu_OS = 0;                    // time spent outside of loop()
int cu_loop = 0;                  // time spinning in loop() finding nothing to do
int cu_other = 0;                 // time spent in "none of the above", i.e. what's left to make up the second
int cu_mqtt = 0;                  // time spent in asynchronous MQTT handling routines

// CPU usage measurements, as integer percents, rounded down, for display purposes
int cu$IMU = 0;                   // time spent in goIMU controlled part of main loop
int cu$wifi = 0;                  // time spent in wifi processing in main loop
int cu$OLED = 0;                  // time spent in goOLED controlled part of main loop
int cu$LED = 0;                   // time spent in goLED controlled part of main loop
int cu$metaData = 0;              // time spent in goMetadata controlled part of main loop
int cu$OS = 0;                    // time spent outside of loop()
int cu$loop = 0;                  // time spinning in loop() finding nothing to do
int cu$other = 0;                 // time spent in "none of the above", i.e. what's left to make up the second
int cu$mqtt = 0;                  // time spent in asynchronous MQTT handling routines

                                  
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
// runbit(14)   1     IRAM_ATTR motorTimerISR
// runbit(15)___1     unused
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

// messageControl structs =========================================================================
typedef struct
{
   boolean active = false;
    #define TARGET_CONSOLE 0
    #define TARGET_MQTT 1
   boolean destination = TARGET_CONSOLE;
   String message = "";
} messageControl;                           // Structure for handling messaging for key objects

static volatile messageControl balTelMsg;   // Object that contains details for controlling balance telemetry messaging
static volatile messageControl healthMsg; // Object that contains details for controlling health messaging

// balanceControl balance struct definition =======================================================
typedef struct
{
   float tilt = 0;                            // forward/backward angle of robot, in degrees, positive is leaning forward, 0 is vertical
   // values for balance.method, the next variable in struct
     #define bm_catchup 1   // method based on catchup distance that would pull wheels under the center of mass
     #define bm_angle 2     // method based on applying correction based in PID applied to angle difference from vertical
     #define bm_initialMethod bm_angle    // use this balancing method to start, initialized in setupIMU
   int method;                                // are we using catchup distance balancing method, or angle based PID (see defs above)
     // values for balance.state, the next variable in struct
     #define bs_sleep 0     // inactive. from lying on back until 30 degrees from vertical
     #define bs_awake 1     // within 30 degrees of initial vertical, but still not active
     #define bs_active 2    // has hit vertical, and is now trying to balance
   int state = 0;                             // state within balancing process. see state definitions just above, as in bs_sleep
   float activeAngle = 1;                     // how close to vertical, in degrees, before you start balancing attempt, & go to bs_active
   float targetAngle = 0;                     // angle we're aiming for, when robot is balanced around center of mass      
   float maxAngleMotorActive = 30;            // Maximum angle the robot can lean at before motors shut off
   bool motorTest = false;                    // whether you're doing motor tests vs. balancing
   int testLeft = 0;        // motorINt value for left wheel while testing
   int testRight = 0;       // motorInt value for right wheel while testing

   // control params, setup in cfgBtMAC, and configurable by MQTT
   int slowTicks;               // # of 20uS timer interrupts per step at slowest practical speed
   int fastTicks;               // # of 20uS timer interrupts per step at fastest practical speed
   int directionMod = 1;        // controls if motor direction needs to be reversed base on motor hardware
   float smoother = 0 ;         // smooth changes in speed by using new = old + smoother * (new - old). smoother=0 > disable smoothing 
   float pid;                   // overall value for "Proportional Integral Derivative (PID)" feedback algorithm
   float pidRaw;                // copy of PID before range checking, for telemetry
   int dataCount = 0;           // number of balance data telemetry messages we've sent
   float pidPGain = 150;        // multiplier for the P part of PID
   float pidIGain = 0;          // multiplier for the I part of PID
   int  pidICount = 0;          // number of recent errors to include in I part of PID
   float pidISum = 0;           // the sum if last pidICount error values, used for I part of PID
   float pidDGain = 0;          // multiplier for the D part of PID
   float pidDSlope = 0;         // slope between last 2 error values, used for D part of PID
   int motorTicks;              // motor speed, i.e. interval between steps in timer ticks
   int lastSpeed = 0;           // memory for above method using smoother
   float angleErr = 0;          // difference between current angle and target angle
   int tmrIMU = 12;             // number of milliseconds between calls to readIMU, and balance calculations
   float errHistory[200] ;                    //  remembered angle errors for calculating I in PID
   float centreOfMassError = attribute.heightCOM; // Distance in inches robot's Centre Of Mass (COM) is away from target
   float distancePercentage;                  // Percentage of COM height away from target
   int steps;                                 // Number of steps that it will take to get to target angle
}  balanceControl;                            // Structure for handling robot balancing calculations
volatile balanceControl balance;             // Object for calculating robot balance



// Define global metadata variables. Used to understand the state of the robot, its peripherals and its environment.
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
   int unknownSetvarCnt = 0;      // Track how many invalid variable names occur in setvar commands    
   //TODO Put datapoint below to use
   long riseTimeMax = 0;                     // Most microseconds it took for the signal rise event to happen
   long riseTimeMin = 0;                     // Least microseconds it took for the signal rise event to happen
   long fallTimeMax = 0;                     // Most microseconds it took for the signal fall event to happen
   long fallTimeMin = 0;                     // Least microseconds it took for the signal fall event to happen
   int delayTimeMax = 0;                     // Most microseconds it took for the delay time event to happen
   int delayTimeMin = 0;                     // Least microseconds it took for the delay time event to happen
}  metadataStructure;                        //Structure for tracking key points of interest regarding robot performance
static volatile metadataStructure health; // Object for tracking metadata about robot performance

// Define flags that are used to track what devices/functions are verified working after start up. Initilize false.
boolean leftOLED_detected = false;
boolean rightOLED_detected = false;
boolean LCD_detected = false;
boolean MPU6050_detected = false;
boolean wifi_connected = false;

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
}  //formatMAC()

/** 
 * @brief Converts a string to upper case
 * @return modified argument string
 *        used to make MQTT commands case insensitive
 *       from: https://stackoverflow.com/questions/735204/convert-a-string-in-c-to-upper-case // Brandon Stewart's post, part 1
 * =============================================================================== */

String StringToUpper(String strToConvert)      // convert the argument string to upper case
{
    std::transform(strToConvert.begin(), strToConvert.end(), strToConvert.begin(), ::toupper);
    return strToConvert;
}

/**
 * @brief ISR for left DRV8825 fault condition
===================================================================================================*/
void IRAM_ATTR leftDRV8825fault()
{
   runbit(0) ;
   health.leftDRVfault++;
} // leftDRV8825fault()

/**
 * @brief ISR for right DRV8825 fault condition
=================================================================================================== */
void IRAM_ATTR rightDRV8825fault()
{
   runbit(1) ;
   health.rightDRVfault++;
}  // rightDRV8825fault()

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
}  //ipToString()

/**
 * @brief Connect to WiFi Access Point 
=================================================================================================== */
void connectToWifi()
{
   runbit(2) ;
   health.wifiConAttemptsCnt++; // Increment the number of attempts made to connect to the Access Point
   AMDP_PRINT("<connectToWiFi> Attempt #");
   AMDP_PRINT(health.wifiConAttemptsCnt);
   AMDP_PRINTLN(" to connect to a WiFi Access Point");
   WiFi.begin(mySSID, myPassword);
}  //connectToWifi()

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
   health.mqttConAttemptsCnt++; // Increment the number of attempts made to connect to the MQTT broker
}  //connectToMqtt()

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
}  //WiFiEvent()

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
      }    //case
      case SYSTEM_EVENT_STA_DISCONNECTED:
      {
         AMDP_PRINTLN("<processWiFiEvent> Lost WiFi connection");
         //      int blockTime  = 10; // https://www.freertos.org/FreeRTOS-timers-xTimerStart.html
         //      xTimerStop(mqttReconnectTimer, blockTime);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi.
         //      Disconnect triggers new connect attempt
         //      xTimerStart(wifiReconnectTimer, blockTime); // Activate wifi timer (which only runs 1 time)
         wifi_connected = false;
         health.wifiDropCnt++; // Increment the number of network drops that have occured
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
         Serial.print(health.wifiConAttemptsCnt);
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
         cmdTopicMQTT = myHostName + MQTTTop_commands;   // Define variable with the full name of the incoming command topic
         /*    not sure if we need definitions below
         balTopicHeadingMQTT = myHostName + MQTT_COMMENT; // Define variabe with the full name of the outgoing balance telemetry heading topic
         balTopicMQTT = myHostName + MQTT_BAL_TEL;  // Define variabe with the full name of the outgoing balance telemetry topic
         metTopicMQTT = myHostName + MQTT_HEALTH_REPLY; // Define variable with full name of the outgoiong metadata topic
         cntlParmHeadingMQTT = myHostName + MQTT_CNTL_PARM_HEAD;  // Define variabe with the full name of the outgoing balance telemetry topic
         cntlParmMQTT = myHostName + MQTT_CNTL_PARM;  // Define variabe with the full name of the outgoing balance telemetry topic

         AMDP_PRINT("<processWiFiEvent> cmdTopicMQTT = ");
         AMDP_PRINTLN(cmdTopicMQTT);
         AMDP_PRINT("<processWiFiEvent> balTopicHeadingMQTT = ");
         AMDP_PRINTLN(balTopicHeadingMQTT);
         AMDP_PRINT("<processWiFiEvent> balTopicMQTT = ");
         AMDP_PRINTLN(balTopicMQTT);
         AMDP_PRINT("<processWiFiEvent> metTopicMQTT = ");
         AMDP_PRINTLN(metTopicMQTT);
         AMDP_PRINT("<processWiFiEvent> cntlParmHeadingMQTT = ");
         AMDP_PRINTLN(cntlParmHeadingMQTT);
         AMDP_PRINT("<processWiFiEvent> cntlParmMQTT = ");
         AMDP_PRINTLN(cntlParmMQTT);
         */
         connectToMqtt();
         break;
      }  //case
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
   int cu_mqCon = micros();             // timestamp for start of this async MQTT routine
   runbit(6) ;
   AMDP_PRINTLN("<onMqttConnect> Connected to MQTT");
   AMDP_PRINT("<onMqttConnect> Session present: ");
   AMDP_PRINTLN(sessionPresent);
   uint16_t packetIdSub = mqttClient.subscribe((char *)cmdTopicMQTT.c_str(), MQTTQos); // QOS can be 0,1 or 2. controlled by MQTTQos parameter
   Serial.print("<onMqttConnect> Subscribing to ");   Serial.print(cmdTopicMQTT);
   Serial.print(" at a QOS of ");                     Serial.print(MQTTQos);
   Serial.print(" with a packetId of ");              Serial.println(packetIdSub);
   cu_mqtt += micros() - cu_mqCon;      // add execution time to MQTT CPU usage display
} //onMqttConnect()

/**
 * @brief Handle disconnecting from an MQTT broker
 * @param reason Reason for disconnect
=================================================================================================== */
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
   int cu_mqDis = micros();            // timestamp for start of MQTT CPU usage
   runbit(7) ;
   AMDP_PRINTLN("<onMqttDisconnect> Disconnected from MQTT");
   health.mqttDropCnt++; // Increment the counter for the number of MQTT connection drops
   health.mqttConAttemptsCnt++;
   if (WiFi.isConnected())
   {
      xTimerStart(mqttReconnectTimer, 0); // Activate mqtt timer (which only runs 1 time)
   }                                     //if
   health.mqttConAttemptsCnt = 0;      // Reset the number of attempts made to connect to the MQTT broker
   cu_mqtt += micros() - cu_mqDis;       // add this routines CPU time to MQTT category
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
   int cu_mqSub = micros();            // timestamp for start of MQTT CPU usage
   runbit(8) ;
   AMDP_PRINTLN("<onMqttSubscribe> Subscribe acknowledged by broker.");
   AMDP_PRINT("<onMqttSubscribe>  PacketId: ");  AMDP_PRINTLN(packetId);
   AMDP_PRINT("<onMqttSubscribe>  QOS: ");       AMDP_PRINTLN(qos);
   cu_mqtt += micros() - cu_mqSub;  // add this to MQTT CPU usage

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
   // skipping adding this tiny bit to MQTT CPU tally
   runbit(9) ;
   AMDP_PRINTLN("Unsubscribe acknowledged.");
   AMDP_PRINT("  packetId: ");
   AMDP_PRINTLN(packetId);
} //onMqttUnsubscribe()

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
   // uint16_t packetIdPub1 = 0;                         // initialize itr to avoid compile errors if DEBUG == false

   String message = String(millis() ) + "," + msg;           // prepend timestamp to message
   String mqttPrefix = String(myHostName) + String(topic);  // prepend robot name to MQTT topic
   // do the publish, using topic that was argument to publish routine
      mqttClient.publish((char *)mqttPrefix.c_str(), MQTTQos, false, (char *)message.c_str()); // QOS 0-2, retain t/f
      AMDP_PRINT2LN("<publishMQTT> publish for topic: ",topic);
} //publishMQTT()


/*
The general format of a published MQTT message is
   bot-ID "/" topic space timestamp comma message

for an event...
   topic = "hthEvt"
   message = evtId,evtSev,evtMsg
where evtId takes values
  0   test event
  1   "should not get here" event
      evtMsg is "routine-name, event-number-routine"
  2   faults seen on motor controllers in last 5 seconds
      evtMsg is "counter for last 5 seconds"
  3   ... to be defined ...

and ecvtSev takes the values
  0   Info:    normal operational event information
  1   Warning: of unusual or unexpected condition event
  3   Error:   circumstances that put continued operation of bot at risk   

*/
void publishEvent(int evtId, int evtSev, String evtMsg)
{  publishMQTT("/hthEvt",String(evtId) +","+ String(evtSev) +","+String(evtMsg));
}


/**
 * @brief Set a control parameter variable to the new value specified in the remote setvar command 
 * @param rCMD Remote command sent from MQTT broker
 * @note No effort is put into verifying that the command is properly formed
 */
void setControlParameter(String rCMD)
{
   int firstComma = rCMD.indexOf(",");
   int secondComma = rCMD.indexOf(",", firstComma + 1);
   int lenVarName = secondComma - firstComma - 1;
   int varNameStart = firstComma;
   varNameStart ++;
   String varName = rCMD.substring(varNameStart, varNameStart + lenVarName);
   String varValue = rCMD.substring(secondComma + 1,rCMD.length());
   AMDP_PRINT("<setControlParameter> rCMD length = ");   AMDP_PRINTLN(rCMD.length());
   AMDP_PRINT("<setControlParameter> firstComma = ");    AMDP_PRINTLN(firstComma);
   AMDP_PRINT("<setControlParameter> varNameStart = ");  AMDP_PRINTLN(varNameStart);
   AMDP_PRINT("<setControlParameter> secondComma = ");   AMDP_PRINTLN(secondComma);
   AMDP_PRINT("<setControlParameter> varName = ");       AMDP_PRINTLN(varName);
   AMDP_PRINT("<setControlParameter> lenVarName = ");    AMDP_PRINTLN(lenVarName);
   AMDP_PRINT("<setControlParameter> varValue = ");      AMDP_PRINTLN(varValue);

   if(varName == "BALANCE.PIDPGAIN") balance.pidPGain = varValue.toFloat();
   else if(varName == "BALANCE.PIDIGAIN") balance.pidIGain = varValue.toFloat();
   else if(varName == "BALANCE.PIDICOUNT") balance.pidICount = varValue.toInt();
   else if(varName == "BALANCE.PIDDGAIN") balance.pidDGain = varValue.toFloat();
   else if(varName == "BALANCE.SLOWTICKS") balance.slowTicks = varValue.toFloat();
   else if(varName == "BALANCE.FASTTICKS") balance.fastTicks = varValue.toFloat();
   else if(varName == "BALANCE.SMOOTHER") balance.smoother = varValue.toFloat();
   else if(varName == "BALANCE.TARGETANGLE") balance.targetAngle = varValue.toFloat();
   else if(varName == "BALANCE.ACTIVEANGLE") balance.activeAngle = varValue.toFloat();
   else if(varName == "BALANCE.TMRIMU") balance.tmrIMU = varValue.toInt();   // be very careful if you change this
  

   // use some special pseudo variables to handle variables with non-numeric values
   else if (varName == "BALTELOFF") balTelMsg.active = false;                                         // value irrelevant
   else if (varName == "BALTELCON")  {balTelMsg.active = true; balTelMsg.destination=TARGET_CONSOLE;} // value irrelevant
   else if (varName == "BALTELMQTT") {balTelMsg.active = true; balTelMsg.destination=TARGET_MQTT; }   // value irrelevant

   else if (varName == "HTHMSGOFF") healthMsg.active = false;                                         // value irrelevant
   else if (varName == "HTHMSGCON")  {healthMsg.active = true; healthMsg.destination=TARGET_CONSOLE;} // value irrelevant
   else if (varName == "HTHMSGMQTT") {healthMsg.active = true; healthMsg.destination=TARGET_MQTT;  }  // value irrelevant

   else
   {
     AMDP_PRINTLN("<setControlParameter> Unknown variable. Ignoring setvar command");
     health.unknownSetvarCnt++; // Increment counter of invalid setvar variable names
   } //else

} //setControlParameter()

/**
 * @brief publish current values of major control parameters, whether ot not they're changeable by MQTT
 * @note  called from checkBalanceState()
 * @note  no longer called from processing of MQTT getvars command, which excludes non MQTT changeable parameters
=================================================================================================== */
void publishParams()                  // publish the control parameters to MQTT
{

   publishMQTT(MQTTTop_shtCom,String(balance.pidPGain) +","+ String(balance.pidIGain) 
   +","+ String(balance.pidICount) +","+ String(balance.pidDGain) 
   +","+ String(balance.slowTicks) +","+ String(balance.fastTicks) +","+String(balance.smoother) +","+String(balance.tmrIMU) 
   +","+ String(balance.targetAngle) +","+ String(balance.activeAngle)+","+ String(MQTTQos));
}

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
 * When WiFi is available and  there is an MQTT broker availabe, TWIPe is always subscribed to the topic {robot name}/commands. All
 * incoming messages from that topic are checked against a known list of commands and get processed accordingly. Commands that are
 * not recognized get logged and ignored.
 * 
 * ## Table of Known Commands
 * | Command                | Description                                                                                            |
 * |:-----------------------|:-----------------------------------------------------------------------------------------------|
 * | setvar                 | followed by variable name, followed by new value |  

 
=================================================================================================== */
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
   int cu_mqMsg = micros();       // timestamp for start of the MQTT processing, to measure CPU usge
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

   String UC_command = tmp;                     // make a copy of the incomming command
   UC_command = StringToUpper(UC_command);      // and convert it to upper case for case insensitive comparisons
   if (UC_command.substring(0,6) == "SETVAR")
   {
      AMDP_PRINTLN("<onMqttMessage> Received remote variable set command");
      setControlParameter(UC_command);
   } //if... setvar

// TODO need to review the MQTT topics used below. "controlParameters" probably inappropriate, 
//      but other definitions have a different structure
   else if(UC_command.substring(0,9) == "GETBALVAR")
   { AMDP_PRINTLN("<onMqttMessage> Received getbalvars remote request for modifyable balance control variables");
     int getbalvarMillis = millis();    // capture time that command was received
     publishMQTT(MQTTTop_balCtl,String(getbalvarMillis) +","+String(balance.pidPGain) +","+ String(balance.pidIGain) 
     +","+ String(balance.pidICount) +","+ String(balance.pidDGain) 
     +","+ String(balance.slowTicks) +","+ String(balance.fastTicks) +","+String(balance.smoother)  
     +","+ String(balance.targetAngle) +","+ String(balance.activeAngle)+","+ String(balance.tmrIMU)  );
   } // if... getbalvar

   else if(UC_command.substring(0,12) == "GETHTHVAR")
   { AMDP_PRINTLN("<onMqttMessage> Received gethealthvars remote request for modifyable health control variables");
      int gethealthvarMillis = millis();  // capture time that command was received
      publishMQTT(MQTTTop_hthCtl,String(gethealthvarMillis) +","+String("no health control variables currently implemented"));
   } // if... gethealthvar

  else if(UC_command.substring(0,12) == "GETHTHTEL")
  {  AMDP_PRINTLN("<onMqttMessage> Received gethealthtel remote request for health telemetry values");
     int gethealthtelMillis = millis();
     publishMQTT(MQTTTop_hthTel,String(gethealthtelMillis) +","+String(health.wifiConAttemptsCnt) +","+ String(health.mqttConAttemptsCnt) 
     +","+ String(health.dmpFifoDataMissingCnt) +","+ String(health.wifiDropCnt) 
     +","+ String(health.mqttDropCnt) +","+ String(health.unknownCmdCnt) +","+String(health.leftDRVfault)  
     +","+ String(health.rightDRVfault) +","+ String(health.unknownSetvarCnt));
/*
  //TODO Put datapoint below to use
  long riseTimeMax = 0;                     // Most microseconds it took for the signal rise event to happen
  long riseTimeMin = 0;                     // Least microseconds it took for the signal rise event to happen
  long fallTimeMax = 0;                     // Most microseconds it took for the signal fall event to happen
  long fallTimeMin = 0;                     // Least microseconds it took for the signal fall event to happen
  int delayTimeMax = 0;                     // Most microseconds it took for the delay time event to happen
  int delayTimeMin = 0;                     // Least microseconds it took for the delay time event to happen
*/
   } // if... gethealthtel

   else if(UC_command.substring(0,5) == "MOTOR")
   {
      int firstComma = tmp.indexOf(",") ;
      int secondComma = tmp.indexOf(",", firstComma + 1) ; 

      int lenLeft = secondComma - firstComma - 1;
      int lenRight = len - secondComma - 1 ;

      String valLeft = tmp.substring(firstComma + 1,firstComma + 1 + lenLeft) ; 
      String valRight = tmp.substring(secondComma + 1, secondComma + 1 + lenRight);

      balance.testLeft = valLeft.toInt();
      balance.testRight = valRight.toInt();
      balance.motorTest = true;                     // guess that we have a non zero speed to turn on test mode
      if (balance.testLeft == 0 && balance.testRight == 0 )   // but check to see if they're zeros
      {   balance.motorTest = false;         // if so, exit from motor test mode...
         //                                   // but stop the motors before you go
         noInterrupts();         // block any motor interrupts while we change control parameters
         left.tickSetting = 0;
         right.tickSetting = 0;
         interrupts();            // Allow other code to update balance variables again                               
      }
      else
      {  balance.motorTest = true;
         noInterrupts();         // block any motor interrupts while we change control parameters
         left.tickSetting = balance.testLeft;
         right.tickSetting = balance.testRight;
         interrupts();            // Allow other code to update balance variables again          
      }
   }  // if ... ="motor"
   else
   {
      AMDP_PRINTLN("<onMqttMessage> Unknown command. Doing nothing");
      health.unknownCmdCnt++; // Increment the counter that tracks how many unknown commands have been received
   }                           //else
   cu_mqtt += micros() - cu_mqMsg; 
} //onMqttMessage()

/**
 * @brief Handle the reciept of a PUBACK message message from MQTT broker
 * @param packetId Unique identifier of the message.
=================================================================================================== */
void onMqttPublish(uint16_t packetId)
{
   // skipping adding this routine's tiny execution time to cu_mqtt
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
   delay(wifiDelay); // give it some time to establish the connection
   while ((WiFi.status() != WL_CONNECTED) && (maxConnectionAttempts > 0))
   {
      delay(wifiDelay);       //  wait between reattempts
      AMDP_PRINT("<connectToNetwork> Re-attempting connection to Access Point. Connect attempt count down = ");
      AMDP_PRINTLN(maxConnectionAttempts);
      AMDP_PRINT("<connectToNetwork>  current Wifi.status() is: ");
      AMDP_PRINTLN(wifiSt[WiFi.status()]);
      int WFs = WiFi.status(); // keep it stable during following tests
      if (WFs == 1 || WFs == 4 || WFs == 5 || WFs == 6 || WFs == 0)
      {
         connectToWifi(); // things went bad enough to need another connect attempt
         delay(wifiDelay);     // give it some time to make connection
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
}  //scanNetworks()

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
 * @brief Control stepping of both stepper motors. 
 * @note ISR for each motor calls this routine with the motor number index.  
 * @param index Which motor the interrupt is for. 0 = right motor, 1 = left motor
 * @param mod Odometer modifier. Handle updating the trip odometer for both polarities (directions)
=================================================================================================== */
void stepMotor(int index, uint mod)
{
/*  needs reworking for new structures
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
    stepperMotor[index].interruptCounter = 0;
  } //if
  else
  {
    stepperMotor[index].interruptCounter++;
  } //else
  */
} //stepMotor()

/** 
 * @brief ISR for stepper motors that drive & balance the robot
 * 
=================================================================================================== */
//TODO Put balance logic in here
void IRAM_ATTR motorTimerISR()
{
   // runbit(14) ;                               //we know we're getting here - optimize ISR
   //                                            // assume catchup method will use same ISR as angle method
   right.tickCounter ++;                         // increment our once per interrupt tick counter
   if(right.tickCounter > right.tickLimit)       // did that take us to the limit value?
   {  right.tickCounter = 0;                     // yes, reset the counter, which goes upwards
      right.tickLimit = right.tickSetting;       // reset upper limit to what the background calculated in balanceByAngle()
      if(right.tickLimit< 0)                     // negative throttle means backwards
      {  digitalWrite(gp_DRV1_DIR,LOW);          // write zero to direction bit on DRV8825 motor controller
         right.tickLimit *= -1;                  // get back to a +ve number for counter comparisons
      }
      else digitalWrite(gp_DRV1_DIR,HIGH);       // if not negative limit value, set wheel direction = forward
   }
   else if(right.tickCounter == 1) digitalWrite(gp_DRV1_STEP,HIGH);  // start the step pulse at end of first counted tick
   else if(right.tickCounter == 2) digitalWrite(gp_DRV1_STEP,LOW);   // end the step pulse at end of second counted tick

   left.tickCounter ++;                          // increment our once per interrupt tick counter
   if(left.tickCounter > left.tickLimit)         // did that take us to the limit value?
   {  left.tickCounter = 0;                      // yes, reset the counter, which goes upwards
      left.tickLimit = left.tickSetting;         // reset upper limit to what the background calculated in balanceByAngle()
      if(left.tickLimit< 0)                      // negative throttle means backwards
      {  digitalWrite(gp_DRV2_DIR,LOW);          // write zero to direction bit on DRV8825 motor controller
         left.tickLimit *= -1;                   // get back to a +ve number for counter comparisons
      }
      else digitalWrite(gp_DRV2_DIR,HIGH);       // if not negative limit value, set wheel direction = forward
   }
   else if(left.tickCounter == 1) digitalWrite(gp_DRV2_STEP,HIGH);  // start the step pulse at end of first counted tick
   else if(left.tickCounter == 2) digitalWrite(gp_DRV2_STEP,LOW);   // end the step pulse at end of second counted tick

} // motorTimerISR()

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
/* needs to be re-worked with new structure names
//de suggest removing function argument, and using stored tilt value
//de  disabling interrupts isn't effective because angle updates are done in background, in readIMU  
  noInterrupts();      // Prevent other code from updating balance variables while we are changing them
  Balance.angleRadians = angleRadians;
  //de following line is already done in readIMU, with out of range protection
  Balance.angleDegrees = Balance.angleRadians * 180 / PI;                        // Convert radians to degrees
  //de  does the math for the following assume vertical is 0 degrees, or 90 degrees?
  Balance.centreOfMassError = attribute.heightCOM - (attribute.heightCOM * sin(Balance.angleRadians)); // Calc distance COM is from 90 degrees
  //de  Balance.COMError = attribute.heightCOM * sin(Balance.tilt * DEG_TO_RAD);     // the catch-up distance
  Balance.steps = Balance.centreOfMassError / attribute.distancePerStep;             // Calc # of steps needed to cover that distance
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
  */
} // calcBalanceParmeters()

/**
 * @brief Adjust motor controls to minimize how far we are from vertical, using PID tuning 
 * called from loop()
=================================================================================================== */
void balanceByAngle()
{
   if(not balance.motorTest )                         // if we're doing PID balancing rather than a speed test, react to current angle
   {
      balance.angleErr = balance.tilt - balance.targetAngle;   // difference between current and desired angles

      // first calculate the P part of PID
      balance.pid = balance.pidPGain * balance.angleErr;       // apply the multiplier for the P in PID, and store in overall PID

      // now  calculate, and add on, the I part = sum of recent error values
      balance.pidISum = 0.;                                    // could be including 0 in I sum, so start with zero sum
      balance.errHistory[0] = balance.angleErr;                // makes the loop a bit easier if it's all in the array
      if(balance.pidICount > 0)                                // unless we're not summing any I values...
      {
         balance.dataCount ++ ;                                // count one more telemmetry message
         int numToSum = balance.dataCount;                     // at startup, don't have b.pidICount values yet
         if (numToSum > balance.pidICount)                     // but will eventually
         {  numToSum = balance.pidICount;                      // max out once you've got enough data
         }
         for(int t=1; t<=numToSum; t++)                        // loop through history for last pidICount error values
         {   balance.pidISum += balance.errHistory[t-1];       // element 0 is the current error, element 1 is previous error....
         }
         balance.pidISum /= numToSum;                          // turn it into the average I value over the history
      }
      #ifdef selectiveISum                                     // if we're selectively avoiding counterproductive ISum values
         if(balance.angleErr * balance.pidISum < 0 )           // if and only if the ISum component is moving us towards vertical
         {   //                                                // ie the current angle error and the ISum have different signs
            balance.pid += balance.pidIGain * balance.pidISum; // add average of stored I values times gain
         }                                                     // but don't make it worse and push bot AWAY from vertical
      #else
         balance.pid += balance.pidIGain * balance.pidISum; // unconditionally add average of stored I values times gain
      #endif

      // now calculate the derivative, which is slope between current and last errors (using errors includes target angle)
      // slope is (delta y) / (delta x), in our case,  (previous error - current error) / (tmrIMU mSec)
      balance.pidDSlope = 0;                  // guess that we won't have enough points to figure slope
      if(balance.pidICount >= 2 )             // but if we do have at least 2 points...
      {  balance.pidDSlope = ( balance.angleErr - balance.errHistory[1] ) / balance.tmrIMU;  // calculate the slope
      }
      // and add that slope, times its gain parameter, to the overall PID
      balance.pid += balance.pidDGain * balance.pidDSlope;

      for(int t = balance.pidICount; t >= 1; t-- )            //shuffle remembered values so we have most recent bunch
      {   balance.errHistory[t] = balance.errHistory[t-1];   
      }

      balance.pidRaw = balance.pid;                           // save a copy of the pid before range checking for telemetry
      if(balance.pid >  400) balance.pid = 400;               // range limit pid
      if(balance.pid < -400) balance.pid = -400;
      if(abs(balance.pid) < 5) balance.pid = 0;              // create a dead band to stop motors when robot is balanced
  /*
    if(balance.pid > 0)
    {   balance.motorTicks = int(balance.slowTicks - (balance.pid/400)*(balance.slowTicks - balance.fastTicks) ) ;  
    }                                      // motorTicks is speed interval in timer ticks
    if(balance.pid < 0)
    {   balance.motorTicks = int( -1*balance.slowTicks - (balance.pid/400)*(balance.slowTicks - balance.fastTicks) ) ;
    }
  */

      float distancePerTick = 3.1415926 * attribute.wheelDiameter / attribute.stepsPerRev;
      float minGroundSpeed = distancePerTick / ( 20 * .000001 * balance.slowTicks);
      float maxGroundSpeed = distancePerTick / ( 20 * .000001 * balance.fastTicks);
      float groundSpeed = 0;  // setting value avoids compiler nagging

      if(balance.pid > 0) { groundSpeed = ((balance.pid-5)/395) * (maxGroundSpeed - minGroundSpeed) + minGroundSpeed; }
      if(balance.pid < 0) { groundSpeed = ((balance.pid+5)/395) * (maxGroundSpeed - minGroundSpeed) - minGroundSpeed; }
      if(balance.pid == 0 || groundSpeed == 0) { balance.motorTicks = 0; }
      // else { balance.motorTicks = int( distancePerTick / groundSpeed ); }
      else { balance.motorTicks = int( distancePerTick / groundSpeed / .000020 ); }

      /*     // some temporary testing stuff
      Serial.print(">");
      Serial.print(balance.tilt); Serial.print(", ");
      Serial.print(balance.pid); Serial.print(", ");
      Serial.print(groundSpeed); Serial.print(", ");
      */

      //   TODO retrofit distancePerTick, groundSpeed, minGroundSpeed, maxGroundSpeed etc, into structures
      // experimental  motor speed change smoothing
      // smoother is a variable, and the control parameter - smoother == 0 means don't smooth
      if(balance.smoother != 0)                       // if smoothing is enabled, do it
      {  balance.motorTicks = balance.lastSpeed + balance.smoother * (balance.motorTicks-balance.lastSpeed);
      }
    
      noInterrupts();         // block any motor interrupts while we change control parameters
      //d2  reverse direction of wheel rotation, based on observation of Dougs bot
      left.tickSetting = balance.directionMod * balance.motorTicks;
      right.tickSetting = balance.directionMod * balance.motorTicks;
      if(balance.lastSpeed * balance.motorTicks < 0 )     // if old and new signs are different, we've reversed desired directions
      {                                 // and we should abort current step in the wrong direction 
         left.tickSetting =  9999 ;    // force counter overflow, and thus reading of the new setting
         right.tickSetting = 9999 ;
      }
      interrupts();
      balance.lastSpeed = balance.motorTicks;                 // remember last speed for smoothing and quick direction change
   }     //if(balance.slowTicks > 0 )

   else   // do this if we are doing motor testing via MQTT motor command rather than IMU controlled balancing
   {    // this is now handled in the main loop() 
   }  // else
  
   // Assemble balance telemetry string

   char flagsInHex[12];                // buffer space for hex string representing runFlagWord
   itoa(runFlagWord,flagsInHex,16);    // convert flags to hex string
   runFlagWord = 0 ;                   // clear flags ASAP, so new routines are seen

   /*
   Layout of balance telemetry. 
   Sample msg:  TwipeB4E62D9EA8F9/balTel 159633,12,1,0,1,2,-0.84,-1.34,-222.57,-222.57,-4.33,-0.01,-521,8001000,0,0,0
   Fields:
   1  Robot identifier, ending in MAC address then a slash separator
   2  MQTT topic "balTel" with space separator
   3  timestamp, in millis() for message publication, followed by a comma separator, like remaining fields
   4  tm_IMUdelta     telemetry value: measured time (millis()) between goIMU calls. should equal tmrIMU
   5  tm_readFIFO     telemetry value: how long the mpu.dmpGetCurrentFIFOPacket(fifoBuffer) execution took
   6  tm_dmpGet       telemetry value: how long the dmpGet* calls after above call took
   7  tm_allReadIMU   telemetry value: how long the readIMU execution took
   8  tm_oldbalByAng  telemetry value: how long the PREVIOUS balanceByAngle took
   9  balance.tilt    forward/backward angle of robot, in degrees, positive is leaning forward, 0 is vertical
   10 balance.angleErr difference between current angle (tilt) and desired angle (targetAngle)
   11 balance.pidRaw  calculated balance angle PID value before range checking
   12 balance.pid     calculated balance angle PID value after range checking (400<pid<400)
   13 balance.pidISum The I part if PID 
   14 balance.pidDSlope  The D part of PID
   15 balance.motorTicks The number of 20usec ticks before next step of the stepper motors by interrupt level
   16 flagsinhex      bit encoded indication of which routines have executed since last readIMU cycle
   17 tm_ROLEDtime    time spent in the routine that updates the right OLED since last readIMU cycle
   18 tm_MQpubCnt     the number of times the MQTTpublish reoutine was executed since last readIMU cycle
   19 tm_uMDtime      telemetry measure: time spent in updateMetadata() since last readIMU cycle

   */

  String tmp = String(tm_IMUdelta) +"," + String(tm_readFIFO) + "," + String(tm_dmpGet) + "," + String(tm_allReadIMU)
   + "," + String(tm_OldbalByAng) + "," + String(balance.tilt) + "," + String(balance.angleErr) + "," + String(balance.pidRaw)
   + "," + String(balance.pid) + "," + String(balance.pidISum) + "," + String(balance.pidDSlope) + "," + String(balance.motorTicks) 
   + "," + flagsInHex  +","+ String(tm_ROLEDtime) +","+ String(tm_MQpubCnt) +","+ String(tm_uMDtime);

   tm_ROLEDtime = 0;         // don't leave old time hanging around in case routine doesn't run soon.
   tm_LOLEDtime = 0;         // reset variables that are counters spanning execuitions of readIMU...
   tm_uMDtime = 0;
   tm_IMUdelta = 0;
   tm_readFIFO = 0;
   tm_dmpGet = 0;
   tm_allReadIMU = 0;
   tm_MQpubCnt = 0;

   if (balTelMsg.active) // If configured to write balance telemetry data
   {
      if (balTelMsg.destination == TARGET_CONSOLE) // If we are to send this data to the console
      {
         Serial.print("<balanceByAngle> ");
         Serial.println(tmp);
      }    //if
      else // Otherwise assume we are to send the data to the MQTT broker
      {
      publishMQTT(MQTTTop_balTel, tmp);              // publish data point string built above.
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
   if (healthMsg.active) // If configured to write metadata
   {
      String tmp = String(health.wifiConAttemptsCnt)
      + "," + String(health.wifiDropCnt)
      + "," + String(health.mqttConAttemptsCnt)
      + "," + String(health.mqttDropCnt)
      + "," + String(health.dmpFifoDataPresentCnt)
      + "," + String(health.dmpFifoDataMissingCnt)
      + "," + String(health.unknownCmdCnt)
      + "," + String(health.leftDRVfault)
      + "," + String(health.rightDRVfault);

      if (healthMsg.destination == TARGET_CONSOLE) // If we are to send this data to the console
      {
         AMDP_PRINT("<updateMetaData> ");
         AMDP_PRINTLN(tmp);
      }    //if
      else // Otherwise assume we are to send the data to the MQTT broker
      {
         publishMQTT(MQTTTop_hthTel, tmp);
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
      /* ---- remove angle display in right OLED to save compute cycles  
      rightOLED.clear();
      rightOLED.drawString(40, 0, String(balance.tilt));
      //    rightOLED.drawString(0, 16, String("Mtr: ")+String(motorInt));    // take this out to see impact on goIMU timing
      //    rightOLED.drawString(0, 32, String("PID: ")+String(pid));
      rightOLED.display();
      ---- end of removed portion   */
      telMilli5 = millis();                  // timestamp between L & R OLED updates
      tm_ROLEDtime = telMilli5 - telMilli4; // telemetry measure - time spent in updateRightOLED
      tm_LOLEDtime = millis() - telMilli5;  // telemetry measure - time spent in updateLeftOLED
      goOLED = millis() + tmrOLED;          // Reset OLED update target time
   }
} //UpdateRightOLED()

/**
 * @brief Update left OLED dipsplay with current routine being executed within setup()
 * @param stage string indicating the stage within setup() we are about to start
 * @note This routine is only called during the initial execution of setup()   
=================================================================================================== */
void updateLeftOLED(String title, String stage)
{
   runbit(20) ;
   leftOLED.clear();
   leftOLED.drawString(0, 0, String(title)+"      ");
   leftOLED.drawString(0, 32, String("> ")+String(stage)+"      ");
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
//  leftOLED.drawString(32, 20, "My Demo"); //64,22
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
      mpu.setXGyroOffset(attribute.XGyroOffset);
      mpu.setYGyroOffset(attribute.YGyroOffset);
      mpu.setZGyroOffset(attribute.ZGyroOffset);
      mpu.setXAccelOffset(attribute.XAccelOffset);
      mpu.setYAccelOffset(attribute.YAccelOffset);
      mpu.setZAccelOffset(attribute.ZAccelOffset);
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
      balance.method = bm_initialMethod;      // are we balancing by catchup distance, or angle deviation?
      balance.dataCount = 0;                 // balance data telemetry message counter
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
   }  //else
} //setupIMU()

/**
 * @brief Set up robot specific configuration based on the ESP32 MAC address
=================================================================================================== */
void cfgByMAC()
{
   tmpHostNameVar = myHostNameSuffix + myMACaddress;
   myMACaddress = formatMAC();
   if (myMACaddress == "B4E62D9E9061") // This is Andrew's bot
   {
      AMDP_PRINTLN("<cfgByMAC> Setting up MAC B4E62D9E9061 configuration - Andrew");
      attribute.XAccelOffset = -4777;
      attribute.YAccelOffset = 1977;
      attribute.ZAccelOffset = 2043;
      attribute.XGyroOffset = 38;
      attribute.YGyroOffset = 17;
      attribute.ZGyroOffset = 3;
/*                                          // values before recalibrate on 210224
      attribute.XGyroOffset = -4691;
      attribute.YGyroOffset = 1935;
      attribute.ZGyroOffset = 1873;
      attribute.XAccelOffset = 16383;
      attribute.YAccelOffset = 0;
      attribute.ZAccelOffset = 0;
*/
      attribute.heightCOM = 5;
      attribute.wheelDiameter = 3.937008; // 100mm in inches
      attribute.stepsPerRev = 200;
      balance.slowTicks=800; //600
      balance.fastTicks=300; //300
      balance.directionMod = -1;
      balance.smoother=0;
      balance.pidPGain=5;
      balance.pidIGain=5;
      balance.pidICount=35;
      balance.pidDGain=0;
      balance.activeAngle=1;
      balance.targetAngle=.75;
      balance.tmrIMU=12;
      MQTT_BROKER_IP = "192.168.2.21";
      balTelMsg.active = true; balTelMsg.destination=TARGET_MQTT;   // simulate BalTelMQTT command
   }                                        //if
   else if (myMACaddress == "B4E62D9EA8F9") // This is Doug's bot
   {
      AMDP_PRINTLN("<cfgByMAC> Setting up MAC B4E62D9EA8F9 configuration - Doug");
      attribute.XAccelOffset = 1815;
      attribute.YAccelOffset = -427;
      attribute.ZAccelOffset = 1725;
      attribute.XGyroOffset = 57;
      attribute.YGyroOffset = -13;
      attribute.ZGyroOffset = 49;
/*                                          // values before recalibrate on 210224
      attribute.XAccelOffset = -2070;
      attribute.YAccelOffset = -70;
      attribute.ZAccelOffset = 1641;
      attribute.XGyroOffset = 60;
      attribute.YGyroOffset = -10;
      attribute.ZGyroOffset = -72;
*/
      attribute.heightCOM = 5;
      attribute.wheelDiameter = 3.937008; // 100mm in inches
      attribute.stepsPerRev = 200;
      balance.slowTicks=800;
      balance.fastTicks=300;
      balance.directionMod = -1;  // changed when started using same Makeblock motors as Andrew
      balance.smoother=0;
      balance.pidPGain=5;
      balance.pidIGain=5;
      balance.pidICount=17;
      balance.pidDGain=0;
      balance.activeAngle=1;
      balance.targetAngle=0;
      balance.tmrIMU=12;
      MQTT_BROKER_IP = "192.168.0.99";
      balTelMsg.active = true; balTelMsg.destination=TARGET_MQTT;   // simulate BalTelMQTT command
    } //else if
   else
   {
      Serial.println("<cfgByMAC> MAC not recognized. Setting up generic configuration");
      attribute.XGyroOffset = 135;
      attribute.YGyroOffset = -9;
      attribute.ZGyroOffset = -85;
      attribute.XAccelOffset = -3396;
      attribute.YAccelOffset = 830;
      attribute.ZAccelOffset = 1890;
      attribute.heightCOM = 5;
      attribute.wheelDiameter = 3.937008; //100mm in inches
      attribute.stepsPerRev = 200;
      balance.slowTicks=600;
      balance.fastTicks=300;
      balance.directionMod = -1;
      balance.smoother=0;
      balance.pidPGain=150;
      balance.pidIGain=0;
      balance.pidICount=0;
      balance.pidDGain=0;
      balance.activeAngle=1;
      balance.targetAngle=0;
      MQTT_BROKER_IP = "unrecognized MAC";
  } //else

   attribute.wheelCircumference = attribute.wheelDiameter * PI;
   attribute.distancePerStep = attribute.wheelCircumference / attribute.stepsPerRev;
   AMDP_PRINT("<cfgByMAC> Wheel circumference = ");
   AMDP_PRINTLN(attribute.wheelCircumference);
   AMDP_PRINT("<cfgByMAC> Distance per step = ");
   AMDP_PRINTLN(attribute.distancePerStep);
  
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
   

      // once a second, update left OLED with CPU utilization information
      if(blinkState == true)          // it alternates - this is a half second timer
//      if(1 == 0)          // it alternates - this is a half second timer
      {
         // TODO rewite OLED display routines, using low frequency display, independent of LED
         rightOLED.clear();
   #ifdef displayCpuUsage
         // String tmp = String("IM:") +String(cu$IMU) +String(" Wi:") +String(cu$wifi) +String(" OL:") +String(cu$OLED)+ String("|");
         String tmp = String("IM:") +String(cu$IMU) +String(" Wi:") +String(cu$wifi) +String(" OL:") +String(cu$OLED)+ String("|");
         rightOLED.drawString(0,0,String(tmp));

         tmp = String("LD:") +String(cu$LED) +String(" MD:") +String(cu$metaData) +String(" OS:") +String(cu$OS)+ String("|");
         rightOLED.drawString(0,16,String(tmp));

         tmp = String("loop:") +String(cu$loop) +String(" othr:") +String(cu$other) + String("|");
         rightOLED.drawString(0,32,String(tmp));
   #endif

         String tmp = String("Mq: ") + String(cu$mqtt) +String("*") + String(health.leftDRVfault) +String("*")+String(health.rightDRVfault) +String("*");
         rightOLED.drawString(0,48,String(tmp));

         rightOLED.display();

      }

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

      balance.tilt = ypr[2] * RAD_TO_DEG - 90. ; // get the Roll, relative to original IMU orientation & adjust
      if (balance.tilt < -180.) balance.tilt = 90.;     // avoid abrupt change from +90 to -270, when he's past a face plant
      health.dmpFifoDataPresentCnt++;          // Track how many times the FIFO pin goes high and the buffer has data in it
      rCode = true;
  }  //if
  else // If sampling rate is reasonable, but no data is available then something weird happened
  {
      health.dmpFifoDataMissingCnt++; // Track how many times the FIFO pin goes high but the buffer is empty
  }                                   //else
  return rCode;
} // readIMU()

/**    
 * @brief Create FreeRTOS timers that run callback functions in their own separate FreeRTOS threads. 
 * @note For details abut FreeRTOS timers see https://www.freertos.org/FreeRTOS-timers-xTimerCreate.html
 * @note Timers are created but are not started at this point. xTimerStart is used later to start them.
=================================================================================================== */
void setupFreeRTOStimers()
{
   runbit(25) ;
   int const wifiTimerPeriod = 2000;                              // Time in milliseconds between wifi timer events
   int const mqttTimerPeriod = 2000;                              // Time in milliseconds between mqtt timer events
   mqttReconnectTimer = xTimerCreate("mqttTimer",                 // Human readable name assigned to timer
      pdMS_TO_TICKS(mqttTimerPeriod),                              // set timer period. pdMS_TO_TICKS() converts milliseconds to ticks
      pdFALSE,                                                     // Set reload to FALSE so this timer becomes dormant after one run
      (void *)0,                                                   // Timer ID. Not used in our callback function
      reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));   // Function the timer calls when it expires
   wifiReconnectTimer = xTimerCreate("wifiTimer",                  // Human readable name assigned to timer
      pdMS_TO_TICKS(wifiTimerPeriod),                              // set timer period. pdMS_TO_TICKS() converts milliseconds to ticks
      pdFALSE,                                                     // Set reload to FALSE so this timer becomes dormant after one run
      (void *)0,                                                   // Timer ID. Not used in our callback function
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
   digitalWrite(gp_DRV1_ENA, HIGH); // Disable left motor

   // Set up GPIO pins for the robot's left motor
   AMDP_PRINTLN("<setupDriverMotors> Initialize GPIO pins for left motor");
   pinMode(gp_DRV2_DIR, OUTPUT);   // Set right direction pin as output
   pinMode(gp_DRV2_STEP, OUTPUT);  // Set right step pin as output
   pinMode(gp_DRV2_ENA, OUTPUT);   // Set right enable pin as output
   pinMode(gp_DRV2_FAULT, INPUT);  // Set right driver fault pin as input
   digitalWrite(gp_DRV2_DIR, LOW); // Set right motor direction as forward
   digitalWrite(gp_DRV2_ENA, HIGH); // Disable Right motor

   // Set up motor driver ISR
   AMDP_PRINTLN("<setupDriverMotors> Configure timer0 to control the motor timing interrupts");
   uint8_t timerNumber = 0;                                               // Timer0 will be used to control the motors
   uint16_t prescaleDivider = 80;                                         // Timer0 uses presaler (divider) of 80 so interrupts occur at 1us
   bool countUp = true;                                                   // Timer0 will count up not down
   motorTimer = timerBegin(timerNumber, prescaleDivider, countUp);        // Set Timer0 configuration
   bool intOnEdge = true;                                                 // Interrupt on rising edge of Timer0 signal
   timerAttachInterrupt(motorTimer, &motorTimerISR, intOnEdge);           // Attach ISR to Timer0
   bool autoReload = true;                                                // Should the ISR timer reload after it runs
   timerAlarmWrite(motorTimer, motorISRus, autoReload);                   // Set up conditions to call ISR
   timerAlarmEnable(motorTimer);                                     // Enable timer interrupt
         
   // Attach interrupts to track DVR8825 faults
   AMDP_PRINTLN("<setupDriverMotors> Monitor left & right DRV8825 drivers for faults");
   attachInterrupt(gp_DRV2_FAULT, leftDRV8825fault, FALLING);
   attachInterrupt(gp_DRV1_FAULT, rightDRV8825fault, FALLING);            // and specify ISR to call

} //setupDriverMotors()

/**
 * @brief Enable or disable motor based on robot angle
=================================================================================================== */
void checkBalanceState()
{
   runbit(27) ;
   // check to see if there's been a change in the balance state, and if so, handle the transition
   switch (balance.state)
   {
      case bs_sleep:
      {
         if( abs(balance.tilt-balance.targetAngle) < balance.maxAngleMotorActive)  // if robot is within 30 degrees of vertical
         {
            if (digitalRead(gp_DRV1_ENA) == HIGH) // If motor is currently turned off
            {
               AMDP_PRINTLN("<checkTiltToActivateMotors> Enable stepper motors");
               digitalWrite(gp_DRV1_ENA, LOW);
            digitalWrite(gp_DRV2_ENA, LOW);
            }  //if
            balance.state = bs_awake;       // we're now waiting to hit almost vertical before going active
            AMDP_PRINTLN( "<checkBalanceState> entering state bs_awake");

            // update left eye with network info that is now available and static
            updateLeftOLEDNetInfo();        // put IP, MAC, Accesspoint & MQTT hostname into left eye.

            // do a test event publish before the stuff that goes into the spreadsheet to avoid messing it up
            publishEvent(0,0,"test-event");

            // publish preliminary info into the MQTT balance telemetry log to help with telemetry interpretation before we get busy
            // first, publish the column titles for the control parameters
            publishMQTT(MQTTTop_shtCom,"PGain,IGain,ICnt,DGain,slow Tks,fast Tks,smooth,tmrIMU,trgt ang,act ang,QOS");

            // then the values for the control parameters
            publishParams();                  // use same routine as MQTT getvars command uses

            // then the column titles for the repeated data points that are published every time we read the IMU and do balancing calculations
            publishMQTT(MQTTTop_shtCom, "IMUdelta,readFIFO,dmpGet,AllReadIMU,OldbalByAng,tilt,angErr,raw pid,pid,Isum,Dslope,MotorInt,runflags,R.O.time,MQpubCnt,uMDtime");

            // the actual data points are published in balanceByAngle()

         }  // if (abs(balance.tilt)

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
         {  if(abs(balance.tilt-balance.targetAngle) <= balance.activeAngle)      // are we almost vertical?
               {  balance.state = bs_active;              // yes, so start trying to balance
                  AMDP_PRINTLN( "<checkBalanceState> entering state bs_active");
                  for (int t =1; t<= balance.pidICount; t++) {balance.errHistory[t] = 0;}  // initialize remembered errors to zero
               }
               if(abs(-balance.targetAngle > balance.maxAngleMotorActive))    // if we're more than 30 degress from vertical...
               {  balance.state = bs_sleep;                                   // fall back to sleep
                  AMDP_PRINTLN("<checkBalanceState> falling back to bs_sleep state");
               }
         break;
      } // case bs_awake

      case bs_active:
      {  if(abs(balance.tilt-balance.targetAngle) >= balance.maxAngleMotorActive)       // have we gone more than 30 degrees from vertical?
         {  balance.state = bs_sleep;    // abort balancing efforts, and go back to waiting for less than 30 degrees tilt
            left.tickSetting = 0;       // stop the motors
            right.tickSetting = 0;
            left.tickLimit = 0;
            right.tickLimit = 0;
            balance.motorTicks = 0;
            AMDP_PRINTLN("<checkTiltToActivateMotors> Disable stepper motors");
            digitalWrite(gp_DRV1_ENA, HIGH);
            digitalWrite(gp_DRV2_ENA, HIGH);
            AMDP_PRINTLN( "<checkBalanceState> entering state bs_sleep");

         }  //  sleep state will look after turning off motor enable
         break;
      } // case bs_active
   } //switch(balance.state)
} //checkBalanceState()

/**
 * @brief Set the robot's objective
 * @param objective String with human readable objective for the robot to pursue
 * ## Table of robot objectives 
 * | Item                     | Details                                                                                               |
 * |:-------------------------|:------------------------------------------------------------------------------------------------------|
 * | stand                    | Tries to maintain an angle of 90 degrees and a COM distance from target of 0 inches |
=================================================================================================== */
void setRobotObjective(int objective)
{
   runbit(28) ;
  /*  needs to be reworked with new structures
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
  */
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
    updateLeftOLED("Setup() stage:          ","cfgByMAC");           // display setup routine we are about to execute in bot's right eye
   cfgByMAC();                            // Use the devices MAC address to make specific configuration settings
    updateLeftOLED("Setup() stage:          ","setRobotObjective");  // display setup routine we are about to execute in bot's right eye
   setRobotObjective(STATE_STAND_GROUND); // Assign robot the goal to stand upright
    updateLeftOLED("Setup() stage:          ","setupLED");           // display setup routine we are about to execute in bot's right eye
   setupLED();                            // Set up the LED that the loop() flashes
    updateLeftOLED("Setup() stage:          ","setupFreeRTOStimers");// display setup routine we are about to execute in bot's right eye
   setupFreeRTOStimers();                 //  User timer based FreeRTOS threads to manage a number of asynchronous tasks
    updateLeftOLED("Setup() stage:          ","setupMQTT");          // display setup routine we are about to execute in bot's right eye
   setupMQTT();                           // Set up MQTT communication
    updateLeftOLED("Setup() stage:          ","setupWiFi");          // display setup routine we are about to execute in bot's right eye
   setupWiFi();                           // Set up WiFi communication
    updateLeftOLED("Setup() stage:          ","setupIMU");           // display setup routine we are about to execute in bot's right eye
   setupIMU();                            // Set up IMU communication
    updateLeftOLED("Setup() stage:          ","setupDriverMotors");  // display setup routine we are about to execute in bot's right eye
   setupDriverMotors();                   // Set up the Stepper motors used to drive the robot motion
    updateLeftOLEDNetInfo();             // aftersetup's done, show IP, MAC, AccessPoint and Hostname in left OLED
   goOLED = millis() + tmrOLED;         // Reset OLED update counter
   goLED = millis() + tmrLED;           // Reset LED flashing counter
   goIMU = millis() + balance.tmrIMU;   // Reset IMU update counterFtitles

   goMETADATA = millis() + tmrMETADATA; // Reset IMU update counter
   updateLeftOLEDNetInfo();             // output network info once since it's stable, not repeatedly

   cu_IMU = 0;                   // time spent in goIMU controlled part of main loop
   cu_wifi = 0;                  // time spent in wifi processing in main loop
   cu_OLED = 0;                  // time spent in goOLED controlled part of main loop
   cu_LED = 0;                   // time spent in goLED controlled part of main loop
   cu_metaData = 0;              // time spent in goMetadata ontrolled part of main loop
   cu_OS = 0;                    // time spent outside of loop()
   cu_loop = 0;                  // time spinning in loop() finding nothing to do
   cu_other = 0;                 // time spent in "none of the above", i.e. what's left to make up the second
   cu_mqtt = 0;                  // time spent in various MQTT processing routines

   cu_secStart = micros();       // cpu utilization is measured over each second, and first second starts now
   cu_lastLoopEnd = 0;           // signal that we're starting, and don't have a previous loop to worry about

   Serial.println(F("<setup> End of setup"));
} //setup()

/**
 * @brief Standard looping routine for Arduino programs
=================================================================================================== */
void loop()
{
  
   cu_loopStart = micros();                     // start point for time used in this loop
   if(cu_lastLoopEnd != 0)                      // if we're doing first measured second, skip adding previous OS overhead
   {
      cu_OS += cu_loopStart - cu_lastLoopEnd;   // add on time after last loop ended & before this one started
   }

   if (millis() >= goIMU)                  // use "else if" to only allow one routine to run per loop()...
   {                                       // which increases frequency of checks for goIMU readiness
      goIMU = millis() + balance.tmrIMU;    // Reset IMU update counter right away.
      holdMilli1 = telMilli1;               // remember previous startime to calculate delta time for goIMU starts
      telMilli1 = millis();                 // get a timestamp for telemetry data (gives telemetry publish delay)
      tm_IMUdelta = telMilli1 - holdMilli1; // telemetry measurement: elapsed time since last goIMU call.
      boolean rCode = readIMU();            // Read the IMU. Balancing and data printing is handled in here as well
      telMilli4 = millis();                 // telemetry timestamp (gives readIMU execution time)
      tm_allReadIMU = telMilli4 - telMilli1; // telemetry measurement: total time for readIMU routine
      if (rCode)                            //de even if we don't read IMU, should still do balancing?
      {
         if(balance.motorTest == true)       // are we in motor testing mode?
         {
            //        AMDP_PRINTLN("<checkTiltToActivateMotors> Enable stepper motors");
            if(digitalRead(gp_SWR_BUTTON) == true)
            {
               // turn motors on, but only if they aren't already on
               if(digitalRead(gp_DRV2_ENA) == HIGH)
               {
                  digitalWrite(gp_DRV1_ENA, LOW);   // turn on the motors
                  digitalWrite(gp_DRV2_ENA, LOW);
               }
               noInterrupts();         // block any motor interrupts while we change control parameters
               left.tickSetting = balance.directionMod * balance.testLeft;   // use motor speeds from MQTT motor command
               right.tickSetting = balance.directionMod * balance.testRight; 
               interrupts();
            }
            else        // i.e. sw readable switch says stop motor test...
            {           // turn off the motors, 
               // leave them enabled, so we don't lose sync with the motors,
               // digitalWrite(gp_DRV1_ENA, HIGH);
               // digitalWrite(gp_DRV2_ENA, HIGH);

               // but stop issuing step commands (leaving the wheels clenched on purpose)
               noInterrupts();         // block any motor interrupts while we change control parameters
               left.tickSetting = 0;   //  leave motor speeds from MQTT motor command untouched
               right.tickSetting = 0;  // but prevent interrupt level from stepping 
               interrupts();
            }  // else gp_SWQR_BUTTON == true
         }  // if(balance.motorTest == true)
         else      // following is normal case where IMU readings control balancing efforts
         {
            checkBalanceState();                // handle balance state changes: sleep, awake, active
 
            if(balance.state ==bs_active)       // if we're in a state where we can try to balance or do speed test
            {                                   // then do so, depending on which method we're using
               if(balance.method == bm_catchup)
               {
                  //de suggest removing the arg in radians, and use stored balance.tilt value in degrees
                  calcBalanceParmeters(ypr[2]);   // Do balancing calculations based on catch up distance
               }  // if(balance.method)
               if(balance.method == bm_angle)
               {  balanceByAngle();                 // Do balancing calc's based on angle displacement from vertical
                  //                                // and publish telemetry, resetting runFlagword
                  telMilli5 = millis();             // telemetry timestamp (gives balanceByAngle execution time)
                  tm_OldbalByAng = telMilli5 - telMilli4; // telemetry measurement: time in BalanceByAngle, reported in NEXT MQTT publish
               }
            }  // if(balance.state...) 
         }   // else , motorTest 
      } // if rCode
      cu_IMU += micros() - cu_loopStart;   // add elapsed cpu time to IMU routine counter
   }  // if millis() > goIMU
   else 
   {  if (WifiLastEvent != -1) 
      {
         processWifiEvent();     // if there's a pending Wifi event, handle it
         cu_wifi += micros() - cu_loopStart;   // add time to wifi routine counter
      }               
      else
      {  if (millis() >= goOLED) 
         {
            updateRightOLED();                    // replace contents of both OLED displays
            cu_OLED += micros() - cu_loopStart;   // add time to OLED routine counter
         }
         else 
         {   
            if (millis() >= goLED)
            {  updateLED();                                          // Update the front amber LED
               if(millis() < 10000) { updateLeftOLEDNetInfo(); } ;    // and the network info in left OLED
               cu_LED += micros() - cu_loopStart;   // add time to LED routine counter
            }            
            else 
            {  if (millis() >= goMETADATA)
               {
                  updateMetaData();           // Send data to serial terminal
                  cu_metaData += micros() - cu_loopStart;   // add time to mettadata routine counter
               }     
               else
               {
                  // here if no routines were executed during loop() - took all the else cases.
                  // capture the loop spinning overhead here, in cu_loop, after end of second work

                  // check to see if we've got to the end of the measurment second
                  int cu_secTime = micros() - cu_secStart;   // how far are we into the current second?
                  if(cu_secTime >= 1000000)                  // are we more than a million microseconds since start of measurement second?
                  {                                          // yes - time to analyse cpu percengtage use & leave it ready for OLED display
                     int cu_subTotal = cu_IMU +cu_wifi +cu_OLED +cu_LED +cu_metaData +cu_OS +cu_loop;
                     // cu_mqtt purposely excluded - it overlaps other usage times

                     cu_other = cu_secTime - cu_subTotal;    //

                     // calculate percent usage and leave it for OLED routines to display
                     // TODO track high water mark for each CPU usage counter
                     cu$IMU = 100*cu_IMU / cu_secTime;                    // % time spent in goIMU controlled part of main loop
                     cu$wifi = 100*cu_wifi / cu_secTime;                  // % time spent in wifi processing in main loop
                     cu$OLED = 100*cu_OLED / cu_secTime;                  // % time spent in goOLED controlled part of main loop
                     cu$LED = 100*cu_LED / cu_secTime;                    // % time spent in goLED controlled part of main loop
                     cu$metaData = 100*cu_metaData / cu_secTime;          // % time spent in goMetadata controlled part of main loop
                     cu$OS = 100*cu_OS / cu_secTime;                      // % time spent outside of loop()
                     cu$loop = 100*cu_loop / cu_secTime;                  // % time spinning in loop() finding nothing to do
                     cu$other = 100*cu_other / cu_secTime;                // % time spent in "none of the above", i.e. what's left to make up the second
                     cu$mqtt = 100*cu_mqtt / cu_secTime;                  // similar %, but it's embedded in other usage times as an interrupting routine

                     cu_IMU = 0;       // zero time counters for next second
                     cu_wifi = 0;
                     cu_OLED = 0; 
                     cu_LED = 0; 
                     cu_metaData = 0;
                     cu_OS = 0; 
                     cu_loop = 0; 
                     cu_other = 0; 
                     cu_mqtt = 0;

                     cu_secStart = micros();          // start up a new measurement second
                  }   // if cu_secTime > 1,000,000 
                  cu_loop += micros() - cu_loopStart;   // add time to loop routine counter

               }    // else for goMETADATA
            }     // else for goLED
         }      // else for goOLED
      }       // else for Wifi event handler
            // get here whether or not a routine was executed during loop
   }
   // set up to capture OS overhead outside of loop() when next loop starts
   cu_lastLoopEnd = micros();           // OS stuff between loops starts at this point, ends at start of next loop() iteration
} //loop()
