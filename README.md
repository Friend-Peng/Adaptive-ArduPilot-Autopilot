# Adaptive-ArduPilot-Autopilot
Adaptive plug-and-play fixed-wing ArduPilot autopilot with boundary-layer (finite-time) control. Please refer to the technical report for the details.

This project illustates a kind of adaptive fixed-wing autopilot which is based on version 4.0.6 of ArduPilot. The features of the new autopilot are:
1. A "plug-and-play" way in ArduPilot. We just plugging an adaptive module into the original code instead of modifying the original PID structure in ArduPilot.
2. Greater robustness and adaptability. The new autopilot framework has greater robustness and adaptability to the uncertainties, such as payload mass change, wind disturbance, etc..

We plug the adaptive module into five controllers, they are TECS throttle demand controller, TECS pitch demand controller, Roll controller, Pitch controller and yaw controller.
![loop](https://github.com/Friend-Peng/Adaptive-ArduPilot-Autopilot/blob/main/loop.jpg)
