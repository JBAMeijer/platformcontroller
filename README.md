
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


**example**
```
./markerdetection -fps 5 -dbg 1 -roi 80
```
## Built With
* [OpenCV](https://opencv.org/) - **OpenCV** (_Open source computer vision_) is a [library of programming functions](https://en.wikipedia.org/wiki/Library_(computing) "Library (computing)") mainly aimed at real-time [computer vision](https://en.wikipedia.org/wiki/Computer_vision "Computer vision").[[1]](https://en.wikipedia.org/wiki/OpenCV#cite_note-1) Originally developed by [Intel](https://en.wikipedia.org/wiki/Intel_Corporation "Intel Corporation"), it was later supported by [Willow Garage](https://en.wikipedia.org/wiki/Willow_Garage "Willow Garage") then Itseez (which was later acquired by Intel[[2]](https://en.wikipedia.org/wiki/OpenCV#cite_note-2)). The library is [cross-platform](https://en.wikipedia.org/wiki/Cross-platform "Cross-platform") and free for use under the [open-source](https://en.wikipedia.org/wiki/Open-source_software "Open-source software") [BSD license](https://en.wikipedia.org/wiki/BSD_license "BSD license").
