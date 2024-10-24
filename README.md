# Aircraft Tracking with Ground-Based Radar

This repository implements two Kalman filters: the traditional **Kalman Filter** and the **Extended Kalman Filter (EKF)** for tracking an aircraft in a vertical plane using ground-based radar measurements. The objective is to estimate the position and velocity of the aircraft over time, based on noisy measurements of its slant distance and angle of elevation.

## Table of Contents
- [Kalman Filter (KF)](#kalman-filter-kf)
- [Extended Kalman Filter (EKF)](#extended-kalman-filter-ekf)
  
---

## Kalman Filter (KF)

### Development of the mathematical models

The traditional Kalman Filter is used to estimate the position and velocity of an aircraft moving with constant velocity, using noisy radar measurements. The mathematical model is based on linear dynamics and position measurements.

* #### State Vector

     The state includes the horizontal position (xx) and horizontal velocity (vx).

* #### Equation of State

     The state transition is linear, assuming rectilinear motion with constant acceleration.

* #### Measurement Equation

     The noisy measurement is only of the horizontal position. (H = \[1 0\])

* #### Noise Covariances

     Process noise affecting acceleration and measurement noise affecting position are considered.
   ```
  matlab
     % Process and sensor covariances
     Q = [0.25*dt^4, 0.5*dt^3; 0.5*dt^3, dt^2] * parm_OB.sigma_acc^2;  % Process covariance (acceleration noise)
     R = parm_OB.sigma_range^2;                                         % Sensor covariance (range noise)

### Results and analysis of the results

The Kalman filter was implemented and the position and velocity estimates were simulated against the true values. Plots were obtained showing the convergence of the estimates as they are updated with new measurements.

### Results and analysis of the results
The Kalman Filter was implemented and simulations were performed to estimate the aircraft position and velocity. The plots show how the estimates converge and match the noisy measurements over time.

![image](https://github.com/user-attachments/assets/7e85ae59-bd97-481d-b703-60e4ca8560fe)

![image](https://github.com/user-attachments/assets/c94e812b-3b1e-4ab3-84ee-8349c3b163c1)

### Conclusions

The Kalman Filter proved to be effective for linear systems, demonstrating that the estimates converge to the true values as they are updated with noisy measurements. This effectiveness highlights its potential for applications in areas such as navigation and tracking, where accurate state estimation is crucial. The simulations conducted support these findings, showing significant improvements in estimation accuracy as more data is incorporated.

---

## Extended Kalman Filter (EKF)

### Development of the mathematical models

The Extended Kalman Filter (EKF) is used to estimate the position and velocity of an aircraft using radar measurements involving an oblique distance and an angle of elevation, which is a non-linear problem.

* #### State Vector

     State includes horizontal position (x), vertical position (y) and velocity (v).

* #### Radar Measurements

     The noisy radar measurements are nonlinearly related to the state through the slant distance r = √((x - xr)² + (y - yr)²) and the elevation angle γ = arctan((y - yr) / (x - xr))

* #### System Model

     The state transition is similar to that of the KF, but a linearisation through the Jacobian is used to implement the EKF.

  ```
  matlab
     % Jacobian of the measurement function
     H = [(mu_pred(1) - x_r)/sqrt((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0, (mu_pred(3) - y_r)/sqrt((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2);
          -(mu_pred(3) - y_r)/((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0, (mu_pred(1) - x_r)/((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2)];
* #### Noise Covariances

     Process noise affecting the velocity (σr = 5 m, σγ = 0.5◦) , and measurement noise affecting the slant distance and elevation angle (σv = 0.02 m s-1, σy = 0.1 m) are considered.

### Results and analysis of the results

The Extended Kalman Filter was implemented and simulations were performed to estimate the aircraft position and velocity. The plots show how the estimates converge and match the noisy measurements over time.

![imagen](https://github.com/mdelicado2021/KF_EKF_AerialRobotics/blob/main/images/images_2/positionx_error2.png)

![imagen](https://github.com/mdelicado2021/KF_EKF_AerialRobotics/blob/main/images/images_2/positiony_error2.png)

![imagen](https://github.com/mdelicado2021/KF_EKF_AerialRobotics/blob/main/images/images_2/vel_error2.png)

![imagen](https://github.com/mdelicado2021/KF_EKF_AerialRobotics/blob/main/images/images_2/innovation2.png)

![imagen](https://github.com/mdelicado2021/KF_EKF_AerialRobotics/blob/main/images/images_2/variance2.png)


### Conclusions

The Extended Kalman Filter proved to be robust in dealing with non-linear systems such as radar measurements. As more measurements are incorporated, the estimates improve significantly, confirming the effectiveness of the EKF for accurate tracking of aircraft position and velocity in a noisy environment.

The Extended Kalman Filter proved to be a powerful tool for dealing with non-linear systems, such as radar measurements. As the filter receives more information, the error decreases significantly, confirming the robustness of the EKF for tracking the position and velocity of an aircraft with noisy measurements.
