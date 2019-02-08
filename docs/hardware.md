# Hardware and wiring

## Hardware - Expansion Board

### Parts
- 1 x **Arduino Pro Mini** or ATMega328P
- 1 x **LM2596 DC-DC Step Down Buck Converter** Module
- 1 x **74HCT4052N multiplexer** resolver. Can be replaced with discrete logic gates as I did in the earlier version [carone-v3.png](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone-v3.png), [data-selector-v3.fzz](https://github.com/olivernadj/carone/raw/master/data-selector-v3.fzz)
- 1 x **AMS1117 3.3 1A Voltage Regulator**
- 4 x 1K, 1 x 10L Resistor
- 2 x 2N2222A NPN Transistor
- 1 x 10nF electrolytic Capacitor
- 4 x 1N4001 Diode - optional

![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone-v4_schem.png)

<p align="center">
  <img width="444" height="444" src="https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/pcb.png">
  <img width="444" height="444" src="https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/pcb-back.png">
</p>
Check out the Fritzing project file: [carone-v4.fzz](https://github.com/olivernadj/carone/raw/master/carone-v4.fzz)
---
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/carone-v4_bb.png)
---

[![Video: CarOne Expansion Boards Evolution - Try and fail](http://img.youtube.com/vi/8EtJPLyAzEU/0.jpg)](https://youtu.be/8EtJPLyAzEU)


## How to wire?
The minimum damage you need to do is to cat wires.

<p align="center">
  <img width="444" height="444" src="https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/cut-the-wires.jpg">
  <img width="444" height="444" src="https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/connect-the-wires.jpg">
</p>

Affected wires on both left and right side:
- Hall Yellow and Blue - for speed detection
- Gyro Sensor Red and Black - for power supply to Expansion board
- Gyro Sensor Green - for receiving fake UART from Expansion board

The transmission signal from the Motherboard to Gyro Sensor is untouched.

Abbreviation | Description |  | Abbreviation | Description
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
![](https://raw.githubusercontent.com/olivernadj/carone/master/docs/pictures/pinout.png)
