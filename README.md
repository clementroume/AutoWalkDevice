# AutoWalkDevice

## Overview

AutoWalkDevice is a preliminary project aimed at creating a lightweight and autonomous device to collect walking data, specifically measuring the time between steps. This project builds on previous academic work (see 
[Almurad et al., 2017](https://www.sciencedirect.com/science/article/abs/pii/S0167945717301288), [Almurad et al., 2018](https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2018.01766/full), and [Ezzina et al., 2020](https://journals.humankinetics.com/view/journals/mcj/25/3/article-p475.xml))  with the goal of developing a more compact, discreet, and efficient solution.

The device will integrate an [accelerometer](https://www.adafruit.com/product/4692), an [SD card](https://www.adafruit.com/product/2922) for data storage, and a [Real-Time Clock (RTC)](https://www.adafruit.com/product/2922), all powered by the [Adafruit Feather ESP32-S3](https://www.adafruit.com/product/5691).

## Components

- [Adafruit Feather ESP32-S3](https://www.adafruit.com/product/5691)
- [ADXL345 Digital Accelerometer](https://www.adafruit.com/product/4692)
- [SD card breakout](https://www.adafruit.com/product/2922) for data logging
- [RTC DS3231 Precision Real-Time Clock](https://www.adafruit.com/product/2922)

## Objectives

- Improve portability and autonomy compared to earlier prototypes.
- Collect precise data on step timing for gait analysis.

## Development

This project is being developed using [PlatformIO](https://platformio.org/), a powerful open-source ecosystem for IoT development. PlatformIO provides an integrated development environment (IDE) that simplifies programming, building, and debugging for embedded systems like the ESP32-S3.

## Installation & Usage

This project is still in its early stages. Future updates will include setup instructions and code for assembling and running the device.

## References

- [Almurad et al., 2017](https://www.sciencedirect.com/science/article/abs/pii/S0167945717301288)
- [Almurad et al., 2018](https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2018.01766/full)
- [Ezzina et al., 2020](https://journals.humankinetics.com/view/journals/mcj/25/3/article-p475.xml)
