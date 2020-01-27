
# Platform Controller
The platform is controlled by an Arduino Due which in turn controls the servo motors attached to the arduino.

## Arduino Mode selector

With the Mode selector, the behaviour of the platform can be changed.
The list below shows the 3 modes and the corresponding action.

| Mode | Led| Description |
| --- | --- | --- |
| 0 | Green  | Manual control mode |
| 1 | Yellow | PID mode |
| 2 | Red    | Demo mode |

### Manual control
With the terminal open, the platform can be controlled with the WASD keys.
WS changes the X-direction of the platform while AD change the Y-direction of the platform.

### PID
The PID mode tries to balance the ball in the center of the plate, by automatically actuating the servo motors, based on the location of the ball, which is received from the UART communication line of the Ball Detector.
The PID-gains can be adjusted via the keyboard, when the terminal is open.

| Gain|Symbol|Increment| Decrement|
| --- | --- | --- | --- |
| Proportional | Kp | 'u' | 'j' |
| Integral | Ki | 'i' | 'k' |
| Derivative | Kd | 'o' | 'l' |

After resetting the programm, the gains are set to their initial values.
In case the gains want to be changed permanently, the constant variables have to be set in the source code.


### DEMO
The Demo mode runs in a loop, controlling the platform in a prefined pattern.


