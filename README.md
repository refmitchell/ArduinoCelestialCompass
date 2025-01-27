# ArduinoCelestialCompass

This repository provides an Arduino interface for the celestial compass hardware from 
[Gkanias et al. (2023)](https://www.nature.com/articles/s44172-023-00132-w).

## Installation and usage

Simply clone the repository into the libraries subdirectory of your Arduino workspace, e.g. 

```
$ cd ~/Arduino/libraries
$ git clone https://github.com/refmitchell/ArduinoCelestialCompass.git
```

Then (re)launch the Arduino IDE and select

```
File -> Examples -> ArduinoCelestialCompass -> TCA9548A_CompassExample
```

This example sketch assumes all eight polarisation opponent units are connected and will read (slowly) from each of them in sequence. 
The example code has been tested on an Arduino Micro. 


## Additional information
The compass is based on eight custom PCBs which each use two ADS112C04 (Texas Instruments) 16-bit analogue to digital converters (ADCs) which read from UV sensitive photodiodes. 
These are multiplexed on a single I2C bus using the TCA9548A (Texas Instruments) I2C multiplexer.

The sensor allows for the measurement of the relative quantities of polarised UV light in different regions of the sky.
For more information on the sensor construction and operational principle, please see [the paper](https://www.nature.com/articles/s44172-023-00132-w).

