# Foot Interface
Introduction 

## Renders


## I/O
| Sensor                 | Description             | Pin   | Notes                                                                                                         |
| ---------------------- | ----------------------- | ----- | ------------------------------------------------------------------------------------------------------------- |
| FSR (Removed)          |                         | AI0   |                                                                                                               |
| FSR                    | Insertion               | AI1   | 33k pulldown                                                                                                  |
| FSR                    | Insertion               | AI2   |                                                                                                               |
| FSR                    | Withdrawal              | AI3   |                                                                                                               |
| FSR                    | Withdrawal              | AI4   |                                                                                                               |
| FSR                    | Withdrawal              | AI5   |                                                                                                               |
| Spare                  | \-                      | AI6   |                                                                                                               |
| Hall Effect            | Servo A CW              | AI7   | Was spare                                                                                                     |
| Hall Effect            | Roll CW                 | AI8   |                                                                                                               |
| Hall Effect            | Roll CCW                | AI9   |                                                                                                               |
| Hall Effect            | Shared Control Switch   | AI10  |                                                                                                               |
| Hall Effect            | Servo A CCW (Gear side) | AI11  |                                                                                                               |
| Removed                | Servo A CW              | AI12  |                                                                                                               |
| Hall Effect            | Servo B CCW             | AI13  | Green/blue wires                                                                                              |
| Hall Effect            | Servo B CW              | AI14  |                                                                                                               |
| Membrane Potentiometer | Pitch                   | AI15  | 0.1 micro cap, 67k resistor<br>Max val: 710<br>Min val: 580<br>Replaced by encoder                            |
| Spare                  |                         | DI1   |                                                                                                               |
| Calibration Switch     | FSRs                    | DI2   |                                                                                                               |
| Calibration LED        | FSRs                    | DI3   |                                                                                                               |
| Spare                  | Servo B                 | DI4   |                                                                                                               |
| Encoder SPI            | Yaw                     | DI5   | Full gear rack protusion: 930<br>Max travel: 859!<br><br>860 - 920<br>Limited to 980                          |
| Encoder SPI            | Yaw                     | DI6   |                                                                                                               |
| Encoder SPI            | Yaw                     | DI7   |                                                                                                               |
| Limit Switch           | E-Stop                  | DI8   | 1 = not pressed (open)<br>Intended for encoder, but full rotation not quite achieved so re-purposed as E-Stop |
| Vibration Motor PWM    |                         | DI9   |                                                                                                               |
| Vibration Motor PWM    |                         | DI10  | \*Funny sound (likely just how mounted?)                                                                      |
| Spare                  | Servo A                 | DI11  |                                                                                                               |
| Servo A PWM            | 11                      | DI12  | Temperamental - sometimes needs to be (ground) plugged in/out<br>Rest: 1500 us                                |
| Servo B PWM            | 4                       | DI13  | Rest: 1500 (can see zero current draw)<br>1478 works too                                                      |
| Spare                  |                         | DI14+ |                                                                                                               |

## Control Philosophy

## Steps to run the node
```Bash
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyACM1
$ rostopic echo /interface_cmd
```
