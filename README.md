# UDACITY-SELF_DRIVING_CAR_ENGINEER_NANODEGREE_project6.-kidnapped-vehicle



[//]: # (Image References)

[image1-1]: ./images/laser_radar.png "raser and radar characterastics"

[image2-1]: ./images/kalman_filter_1.jpg "About Kalman Filter"
[image2-2]: ./images/kalman_filter_2.jpg "About Kalman Filter"
[image2-3]: ./images/kalman_filter_3.jpg "About Kalman Filter"
[image2-4]: ./images/kalman_filter_4.jpg "About Kalman Filter"
[image2-5]: ./images/kalman_filter_5.jpg "About Kalman Filter"

[image3-1]: ./images/extended_kalman_filter_1.jpg "About Extended Kalman Filter"
[image3-2]: ./images/extended_kalman_filter_2.jpg "About Extended Kalman Filter"
[image3-3]: ./images/extended_kalman_filter_3.jpg "About Extended Kalman Filter"
[image3-4]: ./images/extended_kalman_filter_4.jpg "About Extended Kalman Filter"
[image3-5]: ./images/extended_kalman_filter_5.jpg "About Extended Kalman Filter"
[image3-6]: ./images/extended_kalman_filter_6.jpg "About Extended Kalman Filter"
[image3-7]: ./images/extended_kalman_filter_7.jpg "About Extended Kalman Filter"
[image3-8]: ./images/extended_kalman_filter_8.jpg "About Extended Kalman Filter"
[image3-9]: ./images/extended_kalman_filter_9.jpg "About Extended Kalman Filter"

[image4-1]: ./images/code_flow.png "Code Flow"

[image5-1]: ./images/result.png "Result"



# Introduction

The object of this project is to localize car's position

Before project (Kalman filter) is to detect object's using sensor fusion, and this project is detect my position exactly

by also using sensor fusion

Of coulse it is possible localize my position by using GPS, but its range accuracy is more than 1m

We need at least 10cm accuracy so we need more than just GPS

In this project, I will use GPS for initializing first car's position

after than use sensor fusion for detecting landmarks that already know its position

and finally find car's exact position by using particle filter



# Background Learning

For this project, I had to learn principle of Particle-Filter

### 1. Motion models

- Assumption for bicycle model

![alt text][image1-1]



### 2. Particle Filters

<img src="./images/kalman_filter_1.jpg" width="500">
<img src="./images/kalman_filter_2.jpg" width="500">
<img src="./images/kalman_filter_3.jpg" width="500">
<img src="./images/kalman_filter_4.jpg" width="500">
<img src="./images/kalman_filter_5.jpg" width="500">


### 3. Implementation of a Particle Filter

<img src="./images/extended_kalman_filter_1.jpg" width="500">
<img src="./images/extended_kalman_filter_2.jpg" width="500">
<img src="./images/extended_kalman_filter_3.jpg" width="500">
<img src="./images/extended_kalman_filter_4.jpg" width="500">
<img src="./images/extended_kalman_filter_5.jpg" width="500">
<img src="./images/extended_kalman_filter_6.jpg" width="500">
<img src="./images/extended_kalman_filter_7.jpg" width="500">
<img src="./images/extended_kalman_filter_8.jpg" width="500">
<img src="./images/extended_kalman_filter_9.jpg" width="500">



# Content Of This Repo
- ```src``` a directory with the project code
	- ```main.cpp``` : reads in data, calls a function to run the Particle filter, calls a function to calculate Error
    - ```ParticleFilter.cpp``` : describes particle filter functions
    - ```ParticleFilter.h``` : header file for .cpp
    - ```map.h``` : information about landmark position
    - ```json.hpp``` : for using json in main.cpp
    - ```helper_functions.h``` : functions used to in ParticleFilter.cpp



# Summary Of Each File
1. FusionEKF.h

```c++
class FusionEKF {
 public:
  
  // Constructor
  FusionEKF();

  // Destructor. 
  virtual ~FusionEKF();
  
  // It is main function that perform predict next position and calculate position by inputing sensor data
  // Firstly, if it has first data input, we have to initialize data by using ekf_.init() function
  // Secondly, predict next x,P by using only dt, Q noise
  // Finally, calculate current position by using sensor data
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // We need KalmanFilter object for ProcessMeasurement
  KalmanFilter ekf_;

 private:
 
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  // noise constant
  double noise_ax;
  double noise_ay;

};
```

2. kalman-filter.h

```c++
class KalmanFilter {
 
 // Need Tools object to calculate RMSE
 Tools tools;
  
 public:

  // Constructor
  KalmanFilter();

  // Destructor
  virtual ~KalmanFilter();

  // Initializes Kalman filter (x, P, F, H, Hj, R, R_ekf, Q, I)
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &H_in, Eigen::MatrixXd &Hj_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &R_ekf_in, Eigen::MatrixXd &Q_in);

  // Predict position,velocity by using only F matrix (dt)
  void Predict();

  // Calculate position, velocity by using Laser measurement data
  void Update(const Eigen::VectorXd &z);

  // Calculate position, velocity by using Radar measurement data
  void UpdateEKF(const Eigen::VectorXd &z);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;
  
  // measurement matrix for Jacobian
  Eigen::MatrixXd Hj_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
  
  // measurement covariance matrix for RADAR
  Eigen::MatrixXd R_ekf_;
  
  // identity matrix
  Eigen::MatrixXd I_;
  
};
```

3. measurement_package.h

```c++
class MeasurementPackage {
 public:
 
  // Select sensor type LASER/RADAR
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // For calculating dt
  long long timestamp_;

  // It include px,py for LASER, rho, theta, rho_dot for RADAR
  Eigen::VectorXd raw_measurements_;
  
};
```

4. tools.h

```c++
class Tools {
 public:

  // Constructor
  Tools();

  // Destructor
  virtual ~Tools();

  // Calculating RMSE
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  // Calculating Jacobian
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};
```

# Results

I have done one simulation and got this result

px,py means position of x,y

vx, vy means calculated velocity of x,y

![alt text][image5-1]


*Test One*

| Input |   MSE   |
| ----- | ------- |
|  px   | 0.1164 |
|  py   | 0.2811 |
|  vx   | 0.4542 |
|  vy   | 0.7827 |


# Conclusion & Discussion

### 1. About total flow of self-driving car

What do we need to make self driving car?

Firstly, we may need detect surroundings like we see when driving

For that, I learned computer vision, deep learning, sensor fusion at before projects

Secondly, we need to know where am I exactly, in the range of 10cm

This project is at exactly this point

This, particle filter is for localization of my exact position

Thirdly, we need to decide how to react like go straight, turn left, change lanes ...

Next project will cover this subjects






