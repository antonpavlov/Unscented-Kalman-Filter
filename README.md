# Unscented-Kalman-Filter
2D object tracking with the Unscented Kalman filter - Udacity's Self-Driving Car Nanodegree.

## Description ##
The covariance of the [Extended Kalman Filter](https://github.com/antonpavlov/Extended-Kalman-Filter) is propagated through linearization of the underlying non-linear model. In a vast majority of cases this technique shows a great performance. However, in case of highly non-linear <i><b>f</b></i> and <i><b>h</b></i>, the performance of the EKF deteriorates dramatically. The [Wikipedia](https://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter) article describes the UKF as following: "<i>The unscented Kalman filter (UKF) uses a deterministic sampling technique known as the unscented transform (UT) to pick a minimal set of sample points (called sigma points) around the mean. The sigma points are then propagated through the non-linear functions, from which a new mean and covariance estimate are then formed. The resulting filter depends on how the transformed statistics of the UT are calculated and which set of sigma points are used—it should be remarked that is always possible to construct new UKFs in a consistent way. For certain systems, the resulting UKF filter more accurately estimates the true mean and covariance.</i>"

This repository contains the source code of the UKF that estimates a highly non-linear trajectory of an object.

## Requirements ##
In order to successfully build and run the program, the following requirements should be fulfilled:
* `cmake` equal or above version 3.5
* `make` equal or above version 4.1
* `gcc/g++` equal or above version 5.4
* [`uWebSocketIO`](https://github.com/uNetworking/uWebSockets) library

[Initial Udacity's repo](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) is a good starting point for setting up an environment.

## Building and Running the code ##
In order to compile, please execute the following commands in a project folder:
* `cmake ./` - configuration of build; in case of any folder changes, you may want to delete `CMakeChache.txt`
* `make` - actual build
* `./UnscentedKF` - an execution of the program

Program opens the port 4567 (configurable in `src/main.cpp`) and waits for a connection of the simulator described in the Validation section below.

## Validation ##
Once compiled and launched, the program tries to establish a connection with a simulator. The **Term 2 Simulator** software includes a graphical simulator for the 2D object tracking by Unscented Kalman Filter. The simulator can be downloaded here: [https://github.com/udacity/self-driving-car-sim/releases](https://github.com/udacity/self-driving-car-sim/releases)

The contents of this repo were tested in Ubuntu Linux 18.04.



| ![Error  X](https://github.com/antonpavlov/Unscented-Kalman-Filter/blob/master/support/RMSE_X.png) |  ![Error  Y](https://github.com/antonpavlov/Unscented-Kalman-Filter/blob/master/support/RMSE_Y.png) |
|---:|:---:|
| ![Error VX](https://github.com/antonpavlov/Unscented-Kalman-Filter/blob/master/support/RMSE_VX.png)  | ![Error  VY](https://github.com/antonpavlov/Unscented-Kalman-Filter/blob/master/support/RMSE_VY.png)  |


## License ##
All software included in this repository is licensed under MIT license terms. All additional programs and libraries have their owners and are distributed under their respective licenses. This repository contains Udacity's intellectual property. For any inquires on its reuse or commercialization, please contact Udacity at [www.udacity.com](https://www.udacity.com/).
