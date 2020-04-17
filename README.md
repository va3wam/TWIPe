# TWIPe


This project contains the embedded system code used for a Two Wheeled Inverted Pendulum robot. 

## Getting Started

There is a [VA3WAM wiki page](https://github.com/va3wam/va3wam.github.io/wiki) that has been created containing information common to all VA3WAM projects. Everything that you need to set up a VA3WAM compatable work space can be found in the wiki. Alternativey, you can follow the instructions and links below to set up your environment.  

### Prerequisites

The VA3WAM development environment used to make the code in this repository is documented [here](https://github.com/va3wam/va3wam.github.io/wiki/Tools).


### Installing

To get a copy of this project up and running on your local machine for development and testing purposes please follow these [instructions](https://github.com/va3wam/va3wam.github.io/wiki/Software-Version-Control).

## Running the tests

To test the PCB please load [this](https://github.com/va3wam/TWIPeTest) code into your robot and then use the local console terminal to see the results of the canned tests. The following is some smaple output that shows you the seven basic tests that will prove if your circuit is working or not.

** <setup> TEST 1 - SERIAL PORT ============================================================= **
** <setup> If you can read this then the test is successful. The serial circuitry is working! **
** <setup> TEST 2 - I2C BUS ================================================================= **
** <scanBus0> Scan I2C bus 0 looking for devices... **
** <scanBus0> Found 2 device(s). **
** <scanBus1> Scan I2C bus 1 looking for devices... **
** <scanBus1> Found 1 device(s). **
** <setup> TEST 3 - VOLTAGE DIVIDER CIRCUIT ================================================= **
** <setup> Battery level reading = 2640 **
** <setup> TEST 4 - WIFI ==================================================================== **
** <scanNetworks> Scanning for WiFi Access Points. **
** <scanNetworks> Number of networks found: 1 **
** <scanNetworks> Best SSID candidate = xxxx **
** <connectToNetwork> Attempt to connect to an Access Point **
** <connectToNetwork> Connection to network FAILED **
** <setup> TEST 5 - STEPPER MOTORS ========================================================== **
** <setup> Parameters for this test... **
** <setup> Frequency = 167.000000 Hz (period of 0.005988 seconds) **
** <setup> Duty cycle = 50.000000 percent **
** <setup> Uptime = 0.002994 seconds. Downtime = 0.002994 seconds **
** <setup> Are the motors spinning? If so then this test passes **
** <setup> TEST 6 & 7 - SOFTWARE CONTROLLED LED and SWITCH =================================== **
** <setup> Push the AMBER button in/out. Did the AMBER LED turn on/off? **
** <setup> Finally, press the GREEN button in/out once. Did this reset the robot? **

## Deployment

At this time the embedded code in this repository is uploaded via a serial connection using PlatformIO. 

## Built With

* [Visual Studio Code](https://code.visualstudio.com/) - Enhanced text editor
* [PlatformIO](https://platformio.org/) - Embedded programming IDE
* [Sea Monkey](https://www.seamonkey-project.org/) - Internet Application Suite
* [Mosquito](https://mosquitto.org/) - MQTT Broker
* [MQTTfx](http://mqttfx.org/) - MQTT client
* [Doxygen](http://www.doxygen.nl/) - Source code documentation generator   

## Contributing

At the present time this code is only done by close friends so no process for becomming a contibutor has been worked out.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/va3wam/TWIPe/tags).

## Authors

* **Doug Elliott** - *Initial work* 
* **va3wam** - *Initial work* 

See also the list of [contributors](https://github.com/va3wam/TWIPe/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Joop Brokking and his [Balancing Robot](http://www.brokking.net/yabr_main.html)
* Tony DiCola and Adafruit Industries for their MQTT QOS1 library
* Jeff Rowberg for the MPU6050 DMP logic
* Daniel Eichhorn for OLED library


