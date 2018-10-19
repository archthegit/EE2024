# EE2024
EE2024 Project - NUSSpace
In this assignment, we implemented a manned space flight system known as NUSpace. The objective this system is to orbit in space and send data to NUSCloud to ensure the smooth functioning of the system. It sends timely data on its fuel tank, obstacle distance and orientation of the system, with the use of temperature sensor, light sensor and accelerometer respectively.

NUSpace operates in 3 modes:
Stationary Mode: This is the default mode of the rocket when it is in the launch pad. The OLED display shows “STATIONARY”. Only the fuel tank is monitored in this mode using the temperature sensor and one click on SW4 allows the system to go into launch mode after a countdown sequence given that it is safe to transition to the next mode.

Launch Mode: This is the mode in which the rocket is launched towards space. The OLED display shows “LAUNCH”. Both the fuel tank and rocket orientation is monitored. 2 clicks on SW4 helps transition into return mode. The system sends data periodically on the temperature and accelerometer readings to NUSCloud.

Return Mode: This is the mode in which the space shuttle mounted on the rocket is detached and returns to land on earth. The OLED display shows “RETURN”. Only the obstacle detection is observed with the help of the light sensor. And the system sends periodic updates on obstacle detection readings to NUSCloud.

