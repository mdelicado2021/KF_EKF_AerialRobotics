# Aircraft Tracking with Ground-Based Radar

This repository implements an **Extended Kalman Filter (EKF)** to track an aircraft in a vertical plane using ground-based radar measurements. The goal is to estimate the position and velocity of the aircraft over time, based on noisy measurements of its slant range distance and elevation angle.

## Table of Contents
- [Problem Statement](#problem-statement)
- [Mathematical Model](#mathematical-model)
- [Code Implementation](#code-implementation)
- [Results and Analysis](#results-and-analysis)
- [Conclusions](#conclusions)

---

## Problem Statement

We are tasked with tracking an aircraft flying at **constant velocity** and **constant altitude** using a ground-based radar. The radar provides two noisy measurements:

1. **Slant range distance** ($r$): the distance between the radar and the aircraft.
2. **Elevation angle** ($c$): the angle between the radar's position and the aircraft's horizontal position.

The state of the aircraft to be estimated includes:
- Horizontal position ($x$)
- Vertical position ($y$)
- Horizontal velocity ($v_x$)
- Vertical velocity ($v_y$)

Given these measurements, we need to develop a model for real-world simulation and implement an onboard Extended Kalman Filter (EKF) to track the aircraft's position over time.

---

## Mathematical Model

### State Vector

The state vector of the system is defined as:

$$
x = \begin{bmatrix} x \\ v_x \\ y \\ v_y \end{bmatrix}
$$

Where:
- $x$: horizontal position
- $v_x$: horizontal velocity
- $y$: vertical position
- $v_y$: vertical velocity

### Radar Measurements

The radar measurements ($z$) are defined as:

$$
z = \begin{bmatrix} r \\ c \end{bmatrix}
$$

Where:
- $r$ is the slant range distance:

$$
r = \sqrt{(x_r - x)^2 + (y_r - y)^2}
$$

- $c$ is the elevation angle:

$$
c = \arctan\left( \frac{y - y_r}{x - x_r} \right)
$$

### System Model

The state transition model (assuming constant velocity) is linear and given by:

$$
F = \begin{bmatrix}
1 & dt & 0 & 0 \\
0 & 1  & 0 & 0 \\
0 & 0  & 1 & dt \\
0 & 0  & 0 & 1
\end{bmatrix}
$$

Where:
- $dt$ is the time step.

The process noise covariance matrix ($Q$) accounts for small uncertainties in the velocity:

$$
Q = \text{diag}([0.02^2, 0.02^2, 0.1^2, 0.1^2])
$$

---

## Code Implementation

### Initialization

We first initialize the state and covariance matrix:

```matlab
dt = 0.1; % Time step
x0 = [0; 100; 1000; 0]; % Initial state: [x, vx, y, vy]
P0 = diag([100, 25, 500, 100]); % Initial state covariance

% Process noise covariance
Q = diag([0.02^2, 0.02^2, 0.1^2, 0.1^2]);

% Measurement noise covariance
R = diag([5^2, (0.5*pi/180)^2]); % Convert angle to radians
```

### State Prediction

The state is predicted using the system model:

```matlab
F = [1 dt 0  0;
     0  1 0  0;
     0  0 1 dt;
     0  0 0  1];

x_pred = F * x; % Predicted state
P_pred = F * P * F' + Q; % Predicted covariance
```

### Measurement Update

Using radar measurements (range and elevation), we update the state using the Kalman gain:

```matlab
% Jacobian of the measurement function
H = [(x_pred(1) - x_r)/r, 0, (x_pred(3) - y_r)/r, 0;
     -(x_pred(3) - y_r)/(r^2), 0, (x_pred(1) - x_r)/(r^2), 0];

% Measurement innovation
y_k = z_real - h(x_pred);

% Kalman gain
S = H * P_pred * H' + R;
K = P_pred * H' / S;

% Updated state estimate
x = x_pred + K * y_k;

% Updated covariance
P = (eye(4) - K * H) * P_pred;
```

## Results and Analysis

A simulation was run with 100 time steps, assuming the aircraft starts at x0=0 mx0​=0m and y0=1000 my0​=1000m, with an initial velocity of v=100 m/sv=100m/s.

### Position Estimation

Below is a plot that compares the estimated horizontal position xx with the true position:

```matlab
figure;
plot(t, mu_hist(1,:), 'r', t, z_hist(1,:));
xlabel('Time [s]');
ylabel('Position [m]');
legend('Estimated Position', 'True Position');
```

### Error Analysis

The absolute error between the estimated position and the true position is shown in a semilogarithmic plot:

```matlab
figure;
semilogy(t, abs(mu_hist(1,:) - z_hist(1,:)));
xlabel('Time [s]');
ylabel('Absolute Error (Position)');
```

This plot highlights how the error decreases over time as the EKF improves its estimates.

## Conclusions

The Extended Kalman Filter successfully tracked the position of the aircraft using noisy radar measurements. The filter was able to estimate the position and velocity with increasing accuracy over time, as evidenced by the decreasing error in the plots.

### Key Observations:

- The innovation (difference between the predicted and measured state) decreases over time as the filter learns more about the aircraft's motion.
- The use of a non-linear measurement function (slant range and elevation angle) requires the Jacobian to linearize the system around the current estimate.
- The Kalman gain decreases as the filter becomes more confident in its predictions, reflected in the reduction of the covariance.

Overall, the EKF proved to be an effective method for tracking the aircraft's state in a noisy environment.
