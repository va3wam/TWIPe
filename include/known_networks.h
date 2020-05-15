/*************************************************************************************************************************************
 * @file known_networks.h
 * @author va3wam
 * @brief Create a static array of known wifi networks that the program can try to connect to
 * @version 0.0.1
 * @date 2020-04-21
 * @copyright Copyright (c) 2020
 * @note Change history uses Semantic Versioning 
 * @ref https://semver.org/
 * Version YYYY-MM-DD Description
 * ------- ---------- ----------------------------------------------------------------------------------------------------------------
 * 0.0.1   2020-04-21 Include file created
 *************************************************************************************************************************************/
/// TODO Encrypt this file

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Guard against multiple include statements in the project
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef amKnownNetworks_h
#define amKnownNetworks_h

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Include the standard Arduino libraries
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h> // Arduino Core for ESP32. https://github.com/espressif/arduino-esp32

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Define a list of known Access Points SSIDs that the robot expects to find (with associated 
/// passwords). This list of SSIDs and Passwords will be used during attempts to connect to the WiFi 
/// network. Also define variables that hold the SSID and password of the AP that we ultimately select to 
/// connect with.  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int numKnownAPs = 5; // Number of known APs that the Robot knows how to connect to
const String SSID[numKnownAPs] = { "MN_LIVINGROOM", "MN_WORKSHOP_2.4GHz", "MN_DS_OFFICE_2.4GHz", "MN_OUTSIDE", "borfpiggle"};
const String Password[numKnownAPs] = { "5194741299", "5194741299", "5194741299", "5194741299", "de15ab00be"};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// @brief Close out gaurding against multile includes
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
#endif