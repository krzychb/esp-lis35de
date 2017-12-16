# LIS35DE Motion Sensor Driver Example

Communicate by [ESP32](http://espressif.com/en/products/hardware/esp32/overview) with [LIS35DE 3-axis motion sensor](pictures/lis35de.pdf) using SPI interface.

* [LIS35DE Motion Sensor Datasheet](pictures/lis35de.pdf)
* [LIS35DE Breakboard](pictures/kamodmems2_en.pdf)


## Build Status

[![Build Status](https://travis-ci.org/krzychb/esp-lis35de.svg?branch=master)](https://travis-ci.org/krzychb/esp-lis35de)


## Wiring of sensor used in example code

| Signal Name | Sensor | ESP32 |
| :--- | :---: | :---: |
| Slave Select | SS | GPIO22 |
| Serial Clock | SCK | GPIO18 |
| Master Out Slave In | MOSI | GPIO23 |
| Master In Slave Out | MISO | GPIO19 |


## How to run this example

Please follow [ESP32 Get Started](https://esp-idf.readthedocs.io/en/latest/get-started/index.html).


## Example output

```
I (257) LIS35DE Example: Starting example
I (257) LIS35DE Driver: Initialize SPI
I (257) LIS35DE Driver: Initialize the LIS35DE sensor
I (267) LIS35DE Driver: Power up and enable all three axis
I (277) LIS35DE Driver: Check if sensor responds...
I (277) LIS35DE Driver: Sensor responded correctly
I (287) LIS35DE Example: x:  -1, y:   3, z:  53
I (487) LIS35DE Example: x:   1, y:   3, z:  53
I (687) LIS35DE Example: x:   0, y:   4, z:  52
```

## Contribute

Feel free to contribute to the project in any way you like!

If you find any issues with code or description please report them using *Issues* tab above.


## Credits

This repository has been prepared thanks to great work of the following teams and individuals:

* Espressif team that develops and maintains [esp-idf](https://github.com/espressif/esp-idf)  repository
* Designer of a handy breakboard [KAmodMEMS2](pictures/kamodmems2_en.pdf) with 3-axis motion sensor providing SPI and I2C interfaces by [Kamami](http://kamami.com/)


## License

[Apache License Version 2.0, January 2004](LICENSE)