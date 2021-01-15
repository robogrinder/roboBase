# Robobase

Robobase is a costumed program including communicating with stm32 development board and computer vision part using C++ and OpenCV library 

## 1. Installation
install on Jetson Nano platform:
```bash
sh setUp_in_Nano.sh
```
install on ubuntu18.04 desktop platform:
```bash
sh setUp_NOT_in_Nano.sh
```

## 2. Structure
```bash
Robobase
├── camera C SDK Programming Reference Manual.pdf    // development document for camera
├── CMakeLists.txt
├── config.json                                      // configration file
├── README.md                                        // this file
├── RobotBase
│   ├── common.h        
│   ├── main.cpp
│   ├── record_video
│   │   ├── recording_main.cpp
│   │   └── recording_main.h
│   ├── Robogrinder_SDK
│   │   ├── message.h
│   │   ├── serial_port.cpp
│   │   └── serial_port.h
│   ├── ThreadManagement
│   │   ├── Thread_management.cpp
│   │   └── Thread_management.h
│   └── vision
│       ├── autoAim
│       │   ├── Armor.cpp                 
│       │   ├── Armor.h
│       │   ├── autoAim.cpp
│       │   └── autoAim.h
│       ├── bigbuff
│       │   ├── BigbuffDetection.cpp
│       │   ├── BigbuffDetection.h
│       │   ├── pred_algrsm.cpp
│       │   └── pred_algrsm.h
│       ├── cam
│       │   ├── Daheng.cpp
│       │   ├── Daheng.h
│       │   ├── DxImageProc.h
│       │   └── GxIAPI.h
│       └── control.h
├── setUp_in_Nano.sh
└── setUp_NOT_in_Nano.sh
```
## 3. AutoAim Workflow
a. `ArmorDetector` class will process the color scale and grey scale of same image separately and simultaneously,judging if the LED grey scale contour is in the LED color contour inclusively by using function `pointPolygonTest`. Then the qualified grey scale contour will construct the `LED_bar` class, and change the rotation angle from `0 - 360` degree to `-180 - 180` degree.

b. Pairing two LED_bars to construct the `Armor` class. We call this process as the form `possible armors`. Then using the Armor size, the difference between two LED bars in the `Armor` class, and the length-width ratio by using the function `is_suitable_size` as preliminary filters to filtrate the `Armor` class.

c. The average intensity in the area of two light bars on the Armor plate is calculated by function `get_average_intensity`to filtrate the cases with another light bars in the middle of the Armor plate.

d. Optimal match for LED_bars. We define the statement: `factor = error_angle + 0.5 * angle` as the match coefficient. `angle` means the angle difference of two LED_bar if those are not parallel or `0`. The smaller the angle difference is, the more matchable the two LED_bars are. 
The `LED_bar` class has three field variables: `matched`, if it is matched with other LED_bar, `match_index`, matched LED bar index number, and `match_factor` mentioned above.

Iterate every LED_bar to calculate the new `match_factor` and compare to the previous one, and If the new one is smaller, it means that this LED_bar has a batter match relationship. The previous match relationship will be dismissed and new match relationship will be settled down. The `final_armor_list` will store candidate armors after the end of this iteration.

e. A final Armor plate is selected from the `final_armor_list` using the method closest to the image center point  defined in the macro`#define FAST_DISTANCE` in the `control.h`. And the judge the Armor type.

f. The geographic information will be calculated by function `solvePnP` and convert to gimbal message (Yaw and Pitch) and sent it to serial port.

