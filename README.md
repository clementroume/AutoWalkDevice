# AutoWalkDevice

## Overview

AutoWalkDevice is a pioneering project designed to create a lightweight and autonomous device for collecting walking data, with a focus on measuring the time between steps. Inspired by earlier academic work ([Almurad et al., 2017](https://www.sciencedirect.com/science/article/abs/pii/S0167945717301288), [Almurad et al., 2018](https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2018.01766/full), [Ezzina et al., 2020](https://journals.humankinetics.com/view/journals/mcj/25/3/article-p475.xml)), the project aims to deliver a more compact, discreet, and efficient solution for gait analysis.

The device integrates an [LSM6DSO32 Digital Accelerometer](https://www.adafruit.com/product/4692) along with an [SD card with a built-in Real-Time Clock (RTC)](https://www.adafruit.com/product/2922) for data storage and precise timekeeping, all powered by the [Adafruit Feather ESP32-S3](https://www.adafruit.com/product/5691).

## About the Developer

I’m Clément Roume, a Doctor in Human Movement Sciences from the University of Montpellier and a dedicated researcher in physiological complexity and motor control. My academic journey has spanned years of rigorous research, teaching, and hands-on development of innovative projects like AutoWalkDevice. I started my 24-month alternance as a Full Stack Developer in December 2024, expanding my expertise in web technologies while exploring my passion for embedded systems and data analysis—even without a formal background in electronics.

I blend my academic insights with a genuine curiosity for technology, always striving to solve real-world problems through creative and practical solutions. Connect with me on [LinkedIn](https://www.linkedin.com/in/croume/) or explore my projects on [GitHub](https://github.com/clementroume).

## Project Story

The journey of AutoWalkDevice began with a simple yet ambitious idea: to harness modern technology for improved gait analysis. Motivated by a personal interest in biomechanics and wearable technology, I set out to build a device that could accurately capture step timing while remaining unobtrusive. The initial prototype was born out of academic research and several iterations of testing and refinement.

### Implemented Features

- **Accurate Data Collection:** Measures time intervals between steps using high-precision accelerometry.
- **Reliable Data Logging:** Uses an SD card with a built-in RTC to ensure that every data point is timestamped accurately.
- **Efficient Hardware Integration:** Combines multiple components into a compact, power-efficient design powered by the ESP32-S3.

### Features Planned for Future Implementation

- **Real-Time Data Analysis:** Implement on-device processing for immediate feedback.
- **Wireless Connectivity:** Add Bluetooth or Wi-Fi capabilities for live data transmission to mobile devices.
- **Advanced Gait Analysis:** Develop more sophisticated algorithms to derive deeper insights into walking patterns.
- **Improved Ergonomics:** Refine the physical design for better user comfort and usability.

## Challenges Encountered

Developing AutoWalkDevice came with its share of challenges, including:

- **Hardware Integration:** Achieving seamless communication between the accelerometer and the SD card/RTC module required extensive testing and debugging.
- **Power Management:** Balancing performance and energy efficiency was critical to ensure long-term operation in a portable form factor.
- **Miniaturization:** Designing a compact device that remains both robust and user-friendly posed significant engineering challenges.

## Components

The project leverages the following key components:

- **Adafruit Feather ESP32-S3**  
  ![ESP32-S3](https://cdn-shop.adafruit.com/970x728/5691-01.jpg)  
  The central processing unit that powers the device.

- **LSM6DSO32 Digital Accelerometer**  
  ![LSM6DSO32](https://cdn-shop.adafruit.com/970x728/4692-05.jpg)  
  Provides the precise motion data necessary for step timing measurements.

- **Adalogger FeatherWing**  
  ![Adalogger](https://cdn-shop.adafruit.com/970x728/2922-06.jpg)  
  Combines SD card storage with an integrated RTC for accurate data logging.

## Development Environment

This project is developed using [PlatformIO](https://platformio.org/), an open-source ecosystem that streamlines coding, building, and debugging for embedded systems, particularly with the ESP32-S3.

## Installation & Usage

AutoWalkDevice is currently in its early development phase. Future updates will include comprehensive setup instructions, detailed code documentation, and step-by-step assembly guidelines to help users and contributors get started.

## Future Roadmap

- **On-Device Data Processing:** Enhance the device’s ability to analyze data in real time.
- **Wireless Integration:** Implement connectivity options to enable remote monitoring.
- **User Interface Enhancements:** Develop companion applications for a richer user experience.
- **Design Optimization:** Continue refining the hardware for better performance and comfort.

## References

- [Almurad et al., 2017](https://www.sciencedirect.com/science/article/abs/pii/S0167945717301288)
- [Almurad et al., 2018](https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2018.01766/full)
- [Ezzina et al., 2020](https://journals.humankinetics.com/view/journals/mcj/25/3/article-p475.xml)
