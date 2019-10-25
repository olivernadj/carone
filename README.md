# CarOne
Turn a self balancing scooter -Hoverboard- into a kids' electric car.

![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone.gif) ![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone.png)

The main goal of the project is to keep the original functionality and behavior of the self balancing scooter, but in the same time with an extension board enable external control. Also use cheap and well know electrical parts and component.

**NOTE: this repo is about to build an Expansion board on top of original Motherboard**
With CarOne Expansion board the max speed you can get is 142cm/s. In case you are not intend to keep the original functionality and you don't mind to replace the firmware and completely re-purpose the hoverboard, I recommend to check this active repo out: https://github.com/NiklasFauth/hoverboard-firmware-hack

## How It Works?
It's follow [Brew's idea][drewsblog] and it hijacks gyro sensor communication between the **Motherboard** and the **Gyro Sensor board**. It sends fake gyro sensor information to **Motherboard**, but also it reads **Hall effect sensor** to detect the speed. 


- [Hardware and wiring](./docs/hardware.md)
- [Board Specification and Applied Units](./docs/specification.md)
- [CarOne glossary](./docs/glossary.md)

## Achievements on Arduino Micro (ATmega328P 16MHz) board.

- [SoftSerialParallelWrite](https://github.com/olivernadj/SoftSerialParallelWrite) for sending parallel UART signal to the Daughterboard.
- Shmitt Trigger in background process.
- [Auto-cruise algorithm](./docs/autocruise.ipynb) development of Auto-cruise algorithm before C implementation.
- All of the above dots in a single board.


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
