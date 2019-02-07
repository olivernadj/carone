# CarOne
Turn a self balancing scooter -Hoverboard- into a kids' electric car.

![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone.gif)

The main goal of the project is to keep the original functionality and behavior of the self balancing scooter, but in the same time with an extension board enable external control. Also use cheap and well know electrical parts and component.

**NOTE: this repo is about to bould an Expansion board on top of original Motherboard**
With CarOne Expansion board the max speed you can get is 142cm/s. In case you are not intend to keep the original functionality and you don't mindt to replace the firmware and completly repurpose the hoverboard, I recommend to check this active repo out: https://github.com/NiklasFauth/hoverboard-firmware-hack

## How It Works?
It's follow [Brew's idea][drewsblog] and it hijacks gyro sensor communication between the **Motherboard** and the **Gyro Sensor board**. It sends fake gyro sensor information to **Motherboard**, but also reads **Hall effect sensor** to detect the speed. 
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/pinout.png)

The minimum demage you need to do is to cat wires.
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/cut-the-wires.jpg)

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
