# CarOne
Turn a self balancing scooter -Hoverboard- into a kids' electric car.

![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone.gif)

The main goal of the project is to keep the original functionality and behavior of the self balancing scooter, but in the same time with an extension board enable external control. Also use cheap and well know electrical parts and component.

**NOTE: this repo is about to build an Expansion board on top of original Motherboard**
With CarOne Expansion board the max speed you can get is 142cm/s. In case you are not intend to keep the original functionality and you don't mind to replace the firmware and completely re-purpose the hoverboard, I recommend to check this active repo out: https://github.com/NiklasFauth/hoverboard-firmware-hack

## How It Works?
It's follow [Brew's idea][drewsblog] and it hijacks gyro sensor communication between the **Motherboard** and the **Gyro Sensor board**. It sends fake gyro sensor information to **Motherboard**, but also reads **Hall effect sensor** to detect the speed. 

### How to wire?
The minimum demage you need to do is to cat wires.

<p align="center">
  <img width="444" height="444" src="https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/cut-the-wires.jpg">
  <img width="444" height="444" src="https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/connect-the-wires.jpg">
</p>

Affected wires on both left and right side:
- Hall Yellow and Blue - for speed detection
- Gyro Sensor Red and Black - for power supply to Expansion board
- Gyro Sensor Green - for receiving fake UART from Expansion board

The transmission signal from the Motherboard to Gyro Sensor is untouched.

---
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/pinout.png)
---
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone-v4_schem.png)


Schema abbreviation | Description |  | Schema abbreviation | Description
--- | --- | --- | --- | ---
**`DTR`** | Arduino Serial Reset | | **`L14v.In`** | Left Red Gyro Sensor from the Motherboard
**`DTR`** | Arduino Serial Reset | | **`R14v.In`** | Right Red Gyro Sensor Red from the Motherboard
**`RX`** | Arduino Serial RX | | **`L14v.Out`** | Left Red Gyro Sensor Red to Gyro Sensor
**`TX`** | Arduino Serial TX | | **`R14v.Out`** | Right Red Gyro Sensor Red to Gyro Sensor
**`L-Hall-B`** | Left Hall Blue - join | | **`L-Gyro-Rx`** | Left Green Gyro Sensor Red from Gyro Sensor
**`L-Hall-Y`** | Left Hall Yellow - join | | **`R-Gyro-Rx`** | Right Green Gyro Sensor Red from Gyro Sensor
**`R-Hall-B`** | Right Hall Blue - join | | **`L-Gyro-Tx`** | Left Green Gyro Sensor Red to the Motherboard
**`R-Hall-Y`** | Right Hall Yellow - join | | **`R-Gyro-Tx`** | Right Green Gyro Sensor Red to the Motherboard


---
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone-v4_bb.png)
---



- [Board Specification and Applied Units](./docs/specification.md)
- [CarOne glossary](./docs/glossary.md)
- [Auto-cruise algorithm](./docs/autocruise.ipynb)

## License

    Copyright [2019] [oliver nadj]

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

[//]: # (References)
[drewsblog]: http://drewspewsmuse.blogspot.com/2016/06/how-i-hacked-self-balancing-scooter.html
