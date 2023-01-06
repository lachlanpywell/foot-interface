# Foot Interface
Foot interface to provide control over yaw, pitch, roll and insertion/withdrawal movements of a laparoscope.

## Renders
Refer to the renders folder. 

## I/O
| Sensor              | Description           | Pin  | Notes        |
|---------------------|-----------------------|------|--------------|
| FSR                 | Insertion             | AI0  | 33k pulldown |
| FSR                 | Insertion             | AI1  |              |
| FSR                 | Insertion             | AI2  |              |
| FSR                 | Withdrawal            | AI3  |              |
| FSR                 | Withdrawal            | AI4  |              |
| FSR                 | Withdrawal            | AI5  |              |
| Spare               | -                     | AI6  |              |
| Spare               | -                     | AI7  |              |
| Hall Effect         | Roll CW               | AI8  |              |
| Hall Effect         | Roll CCW              | AI9  |              |
| Hall Effect         | Shared Control Switch | AI10 |              |
| Spare               | -                     | AI11 |              |
| Spare               | -                     | AI12 |              |
| Spare               | -                     | AI13 |              |
| Spare               | -                     | AI14 |              |
| Spare               | -                     | AI15 |              |
| Spare               |                       | DI1  |              |
| Switch              | -                     | DI2  |              |
| LED                 | -                     | DI3  |              |
| Spare               | -                     | DI4  |              |
| Encoder SPI         | Yaw                   | DI5  |              |
| Encoder SPI         | Yaw                   | DI6  |              |
| Encoder SPI         | Yaw                   | DI7  |              |
| Vibration Motor PWM | -                     | DI9  |              |
| Vibration Motor PWM | -                     | DI10 |              |
| Spare               | -                     | DI11 |              |
| Spare               | -                     | DI12 |              |
| Spare               | -                     | DI13 |              |
| Limit Switch        | -                     | DI14 |              |
| Limit Switch        | -                     | DI15 |              |
| Limit Switch        | -                     | DI16 |              |
| Limit Switch        | -                     | DI17 |              |
| Spare               | -                     | DI18 |              |
| Spare               | -                     | DI19 |              |
| Spare               | -                     | DI20 |              |
| Spare               | -                     | DI21 |              |
| Spare               | -                     | DI22 |              |
| Spare               | -                     | DI23 |              |
| Spare               | -                     | DI24 |              |
| Spare               | -                     | DI25 |              |
| Spare               | -                     | DI26 |              |
| Spare               | -                     | DI27 |              |
| Encoder             | Pitch                 | DI28 |              |
| Spare               | -                     | DI29 |              |
| Encoder             | Pitch                 | DI30 |              |
| Spare               | -                     | DI31 |              |
| Encoder             | Pitch                 | DI32 |              |
| Spare               | -                     | DI33 |              |
| Encoder             | Pitch                 | DI34 |              |
| Spare               | -                     | DI35 |              |
| Encoder             | Pitch                 | DI36 |              |
| Spare               | -                     | DI37 |              |
| Encoder             | Pitch                 | DI38 |              |
| Spare               | -                     | DI39 |              |
| Encoder             | Pitch                 | DI40 |              |
| Spare               | -                     | DI41 |              |
| Encoder             | Pitch                 | DI42 |              |                                                                                                         |

## Control Philosophy
*Insertion/withdrawal* movements are actuated via a douple tap of the ball and heel of the foot respectively. The movement is tracked using three force sensing resistors on each side of the foot, with the average reading at each end calculated and used to determine the active command. The command is maintained whilst the the pressure remains depressed from the second tap. To activate the faster speed lift the opposite side of the foot whilst the other side remains in contact with the sensor. 

*Roll* movements are actuated by depressing the rubber lined steel plate. This movement is tracked using hall effect sensors. 

*Shared control* mode (future works) - a hall effect sensor has been wired and configured to act as a toggle switch to enable/disable shared control. 

*Pitch and Yaw* movements are mechanically actuated by sliding the pedal on the linear guide rails. Movements are tracked using rotary encoders. Variable force feedback, locking and self-centering is intended to be provided using stepper motors, coupled through a series elastic actuator and driving a gear rack and pinion system. 

## Steps to run the node
```Bash
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyACM1
$ rostopic echo /interface_cmd
```

## Remaining Items
+ Paint (powder coat) steel 'pedal' plate
+ Drill 2 x 2.2mm hole in overhanging steel plate for limit switch mounting
+ Reprint SEA (if resolving the clearance hole issues is inadequate - redesign if the print does not return to the same equilibrium when the springs are exactly the same length) and assemble
+ Mount force sensing resistors, attach 'pedal' to linear guide blocks and cover with rubber adhesive mat
+ Mount encoders, motors and cantilevered gear rack
+ Program vibration motors, LED and switch (the vibration motors are powered by AA batteries and controlled using Arduino digital pins via a MOSFET).
+ Program stepper motors (recommend using the AccelStepper library, there is multicore wire to be used for wiring the motors to the driver and Arduino. Each driver should be wired to a separate input on the power supply. The driver can be wired in a common anode configuration, that is the enable pins are left floating, PUL+ and DIR+ wired to 5V and PUL- and DIR- wired to Arduino digital pins. Also ensure the dip switch current limit (2.7A) and microstep (recommend 400 pulse/rev due to the significant decline in holding torque beyond this, see https://www.machinedesign.com/archive/article/21812154/microstepping-myths). 
