# **Sensor Fusion: Autonomous Vehicle Object-Tracking Using Extended Kalman Filter (EKF) in C++**
---
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)
[image1]: ./images/EKF_process_flow.png "EKF Process Flow"
[image2]: ./images/data_file_sample.png "Data File Sample"
[image3]: ./images/LR_sim_output_dataset1.png "LR Sim Output Dataset1"
[image4]: ./images/LR_sim_output_dataset2.png "LR Sim Output Dataset2"
[image5]: ./images/L_sim_output_dataset1.png "Laser Sim Output Dataset1"
[image6]: ./images/R_sim_output_dataset1.png "Radar Sim Output Dataset1"
[image7]: ./images/L_sim_output_dataset2.png "Laser Sim Output Dataset2"
[image8]: ./images/R_sim_output_dataset2.png "Radar Sim Output Dataset2"


My goal in this project was to develope a Sensor Fusion module by implementing an **Extended Kalman Filter (EKF) algorithm in C++**, to track and predict a bicycle's position and velocity around a vehicle. The project accomplishes this by using a combination of Standard and Extended Kalman Filters to combine LIDAR and RADAR sensor data, in order to estimate the state of a moving object of interest with noisy lidar and radar measurements.

The LIDAR and RADAR data are provided as inputs by a Udacity Simulator, which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The communication between the Simulator (Client) and the Sensor Fusion Module (Server) happens using [uWebSocketIO](https://github.com/uWebSockets/uWebSockets).

I evaluated state-prediction performance on two datasets, using Root Mean Squared Error (RMSE) on the accumulated error residuals between my object state estimate and the ground truth. Below are the final performance results:

**State-Prediction Performance**

| Object State Variable  | Dataset #1 (RMSE) 		| Dataset #2 (RMSE)|
|:----------------:|:---------------------:|:-------------:| 
| x-position (px) | 0.0964 	|	0.0727	|
| y-position (py) | 0.0853 	|	0.0968	|
| x-velocity (vx) | 0.4154 	|	0.4893	|
| y-velocity (vy) | 0.4316 	|	0.5078	|


## **Project Overview**
---

**Kindly note that this project is developed keeping in mind the Udacity Project Instructions and [Rubric](https://review.udacity.com/#!/rubrics/748/view).*

### **Source Code**
The source code for the project can be found under the [src/](src/) folder. The developed Sensor Fusion module consists of the below main components:
1. FusionEKF (FusionEKF.h, FusionEKF.cpp)

	Handles the fusion by setting the appropriate state and process variables, based on whether the incoming data are from lidar or radar sensor, and uses the KalmanFilter component to estimate object state and update the measurements.

2. KalmanFilter (kalman_filter.h, kalman_filter.cpp)

	Implements the Standard and Extended KalmanFilter algorithms.

3. Tools (tools.h, tools.cpp)

	Implements utility functions to calculate the RMSE and Taylor Series Expansion Jacobian.

4. Simulator and Fusion Module Communication (main.cpp, measurement_package.h)

	Uses [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) to establish communication between Simulator and Fusion Module, relays sensor inputs to FusionEKF for processing, uses Tools for RMSE calculation, and communicates the results back to the simulator.

### **Sensor Fusion General Flow**
---
The overall Sensor Fusion processing flow for state prediction and measurement update is implemented according to the block diagram below:

![alt text][image1]

### **Sensor Input Data**
---
The Simulator uses 2 datasets with data files in the below format, and feeds main.cpp values from it one line at a time.

**Data File Sample**
![alt text][image2]

Each row represents a sensor measurement where the first column tells you if the measurement comes from LIDAR (L) or RADAR (R).

For a row containing lidar data, the columns are:
*sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth*

For a row containing radar data, the columns are: *sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth*

Lidar has two measurements (x, y), while radar has three measurements (rho, phi, rhodot).

The `groundtruth` values represent the actual path the bicycle took and is used for calculating the Root Mean Squared Error (RMSE). The `yaw` inputs are currently ignored.

### **Data Communication Protocol**
---
The main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator is as below:

**INPUT:** values provided by the Simulator to the C++ program (Sensor Fusion Module).

["sensor_measurement"] &rightarrow; the measurement that the simulator observed (either lidar or radar)

**OUTPUT:** values provided by the C++ program (Sensor Fusion Module) to the Simulator.

["estimate_x"] &leftarrow; kalman filter estimated position x

["estimate_y"] &leftarrow; kalman filter estimated position y

["rmse_x"] ["rmse_y"] ["rmse_vx"] ["rmse_vy"]

### **Sensor Fusion - Final Output**
---
The images below show the outputs and RMSEs of the Simulator runs using the developed Sensor Fusion module, on Dataset #1 and Dataset #2:

*Legend:*
* Lidar measurements - Red circles
* Radar measurements - Blue circles with an arrow pointing in the direction of the observed angle
* Estimation markers - Green triangles.

**Simulator Output**

Dataset #1 | Dataset #2
:---------:|:----------:
![][image3]|![][image4]

**RMSE** (also seen in the above output images)

| Object State Variable  | Dataset #1 (RMSE) 		| Dataset #2 (RMSE)|
|:----------------:|:---------------------:|:-------------:| 
| x-position (px) | 0.0964 	|	0.0727	|
| y-position (py) | 0.0853 	|	0.0968	|
| x-velocity (vx) | 0.4154 	|	0.4893	|
| y-velocity (vy) | 0.4316 	|	0.5078	|

### **LIDAR Vs. RADAR Performance**
---
I also ran a quick performance comparision with using only LIDAR and only RADAR measurements on both the datasets. Below are the simulation outputs and RMSEs: 

**Simulator Output - Dataset #1**

Lidar Only | Radar Only
:---------:|:----------:
![][image5]|![][image6]

**RMSE on Dataset #1** (also seen in the above output images)

| Object State Variable  | Lidar Only (RMSE) 		| Radar Only (RMSE)|
|:----------------:|:---------------------:|:-------------:| 
| x-position (px) | 0.1838 	|	0.2320	|
| y-position (py) | 0.1542 	|	0.3354	|
| x-velocity (vx) | 0.5748 	|	0.5697	|
| y-velocity (vy) | 0.4893 	|	0.6841	|

**Simulator Output - Dataset #2**

Lidar Only | Radar Only
:---------:|:----------:
![][image7]|![][image8]

**RMSE on Dataset #2** (also seen in the above output images)

| Object State Variable  | Lidar Only (RMSE) 		| Radar Only (RMSE)|
|:----------------:|:---------------------:|:-------------:| 
| x-position (px) | 0.1673 	|	0.2391	|
| y-position (py) | 0.1567 	|	0.3373	|
| x-velocity (vx) | 0.6566 	|	0.6422	|
| y-velocity (vy) | 0.5001 	|	0.7739	|

We can see in the comparisions above that, due to better spatial resolution than that of a RADAR, the LIDAR tracking shows lower RMSEs for x-position (px) and y-position (py).

Though, as seen in the previous `Sensor Fusion - Final Output` section, Sensor Fusion, which uses both LIDAR and RADAR measurements, shows much better tracking accuracy than the individual sensors.