# TWIPe


This project contains the embedded system code used for a Two Wheeled Inverted Pendulum robot. 

## Getting Started

There is a [VA3WAM wiki page](https://github.com/va3wam/va3wam.github.io/wiki) that has been created containing information common to all VA3WAM projects. Everything that you need to set up a VA3WAM compatable work space can be found in the wiki. Alternativey, you can follow the instructions and links below to set up your environment. Here are the high level steps needed to get started with your very own TWIPe robot:

<ol>
<li>Connect the robot to your computer via a USB cable</li>
<li>Test the robot's circuit. Code and instructions [here](https://github.com/va3wam/TWIPeTest)</li>
<li>Get the calibration values. Code and instructions [here](https://github.com/va3wam/TWIPeCalibrate)</li>
<li>Load up the TWIPe code (with the updated offset values if you did that step)</li>
<li>Put the robot face down parallel to the ground</li>
<li>Let it boot up</li>
</ol>

### Prerequisites

The VA3WAM development environment used to make the code in this repository is documented [here](https://github.com/va3wam/va3wam.github.io/wiki/Tools). To get the best results you need to run the [calibration program](https://github.com/va3wam/TWIPeCalibrate) first. Following the instrcutions provided in the readme file before proceeding with this program.

### Installing

To get a copy of this project up and running on your local machine for development and testing purposes please follow these [instructions](https://github.com/va3wam/va3wam.github.io/wiki/Software-Version-Control).

## Running the tests
You may wish to test the robot circuit or test the code that runs on the circuit.

### Test the Circuit
To test the PCB please load [this](https://github.com/va3wam/TWIPeTest) code into your robot and then folllow the test instructions found in that repository. 

### Calibrate the MPU6050
In order to get proper balancing values from the MPU6050 you must firt run [this](https://github.com/va3wam/TWIPeCalibrate) calibration code then update this code with the seed values produced.

### Test this Code
We have not yet developed a test methodology for the robot code.

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


