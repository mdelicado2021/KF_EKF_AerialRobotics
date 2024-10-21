# Aircraft Tracking with Ground-Based Radar

Este repositorio implementa dos filtros de Kalman: el **Filtro de Kalman** tradicional y el **Filtro de Kalman Extendido (EKF)** para el seguimiento de una aeronave en un plano vertical utilizando mediciones de radar en tierra. El objetivo es estimar la posición y velocidad de la aeronave a lo largo del tiempo, basado en mediciones ruidosas de su distancia oblicua y ángulo de elevación.

## Table of Contents
- [Problem Statement](#problem-statement)
- [Mathematical Model](#mathematical-model)
- [Code Implementation](#code-implementation)
- [Results and Analysis](#results-and-analysis)
- [Conclusions](#conclusions)

---

## Problem Statement

Se nos asigna la tarea de rastrear una aeronave que vuela a **velocidad constante** y **altitud constante** usando un radar basado en tierra. El radar proporciona dos mediciones ruidosas:

1. **Distancia oblicua (slant range)** ($r$): la distancia entre el radar y la aeronave.
2. **Ángulo de elevación** ($c$): el ángulo entre la posición del radar y la posición horizontal de la aeronave.

El estado de la aeronave que queremos estimar incluye:
- Posición horizontal ($x$)
- Posición vertical ($y$)
- Velocidad horizontal ($v_x$)
- Velocidad vertical ($v_y$)

Dadas estas mediciones, necesitamos desarrollar un modelo de simulación y aplicar dos filtros de Kalman para rastrear la posición de la aeronave a lo largo del tiempo: uno lineal (Filtro de Kalman) y uno no lineal (Filtro de Kalman Extendido).

---

## Mathematical Model

### Filtro de Kalman (KF)

Para el **Filtro de Kalman** tradicional, asumimos un movimiento en línea recta con una entrada de aceleración constante. Este modelo es adecuado para sistemas con dinámica lineal, como un movimiento con aceleración constante.

#### Vector de Estado

El vector de estado para el filtro de Kalman es:

$$
x = \begin{bmatrix} p \\ v \end{bmatrix}
$$

Donde:
- $p$: posición
- $v$: velocidad

#### Ecuación de Estado

La ecuación de transición de estado (dinámica) es lineal:

$$
x_{k+1} = A x_k + B u_k
$$

Con:

$$
A = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix}, \quad B = \begin{bmatrix} 0.5 \cdot dt^2 \\ dt \end{bmatrix}
$$

#### Ecuación de Medición

El sensor mide solo la posición, así que la ecuación de medición es:

$$
z_k = H x_k
$$

Con:

$$
H = \begin{bmatrix} 1 & 0 \end{bmatrix}
$$

#### Covarianzas de Ruido

El filtro de Kalman usa dos fuentes de incertidumbre:
- **Ruido del proceso** ($Q$): Afecta a la aceleración.
- **Ruido de la medición** ($R$): Afecta a las mediciones de la posición.

### Filtro de Kalman Extendido (EKF)

El **Filtro de Kalman Extendido** maneja el problema no lineal de las mediciones de radar, que involucran la distancia oblicua ($r$) y el ángulo de elevación ($c$).

#### Vector de Estado

El vector de estado es:

$$
x = \begin{bmatrix} x \\ v_x \\ y \\ v_y \end{bmatrix}
$$

Donde:
- $x$: posición horizontal
- $v_x$: velocidad horizontal
- $y$: posición vertical
- $v_y$: velocidad vertical

#### Mediciones de Radar

Las mediciones de radar están relacionadas con el estado de manera no lineal:

$$
r = \sqrt{(x_r - x)^2 + (y_r - y)^2}
$$

$$
c = \arctan\left( \frac{y - y_r}{x - x_r} \right)
$$

#### Modelo del Sistema

La transición de estado es similar a la del Filtro de Kalman, pero las ecuaciones de medición son no lineales, por lo que se utiliza la **linealización** a través del Jacobiano ($H$) para aplicar el EKF.

---

## Code Implementation

### Kalman Filter (KF)

#### Inicialización

```matlab
dt = 0.1; % Duración del paso de tiempo
x0 = [10; 10]; % Estado inicial: [posición; velocidad]
P0 = [1 0; 0 1]; % Covarianza inicial

Q = [1^2, 0; 0, 1^2]; % Covarianza del ruido del proceso (aceleración)
R = 1^2; % Covarianza del ruido de la medición (rango)
```

#### Predicción del Estado

```
A = [1 dt; 0 1];
B = [0.5*dt^2; dt];

% Predicción del estado
x_pred = A * x + B * u;
P_pred = A * P * A' + Q;
```

#### Actualización con las Mediciones

```
H = [1 0];

% Innovación
y_k = z_real - H * x_pred;

% Ganancia de Kalman
S = H * P_pred * H' + R;
K = P_pred * H' / S;

% Estado actualizado
x = x_pred + K * y_k;

% Covarianza actualizada
P = (eye(2) - K * H) * P_pred;
```

### Extended Kalman Filter (EKF)

#### Inicialización

```
x0 = [0; 100; 1000; 0]; % Estado inicial: [x, vx, y, vy]
P0 = diag([100, 25, 500, 100]); % Covarianza inicial

Q = diag([0.02^2, 0.02^2, 0.1^2, 0.1^2]); % Covarianza del ruido del proceso
R = diag([5^2, (0.5*pi/180)^2]); % Covarianza del ruido de medición (con ángulo en radianes)
```

#### Predicción del Estado

```
F = [1 dt 0  0;
     0  1 0  0;
     0  0 1 dt;
     0  0 0  1];

x_pred = F * x; % Estado predicho
P_pred = F * P * F' + Q; % Covarianza predicha

```

#### Actualización con las Mediciones

```
% Jacobiano de la función de medición
H = [(x_pred(1) - x_r)/r, 0, (x_pred(3) - y_r)/r, 0;
     -(x_pred(3) - y_r)/(r^2), 0, (x_pred(1) - x_r)/(r^2), 0];

% Innovación
y_k = z_real - h(x_pred);

% Ganancia de Kalman
S = H * P_pred * H' + R;
K = P_pred * H' / S;

% Estado actualizado
x = x_pred + K * y_k;

% Covarianza actualizada
P = (eye(4) - K * H) * P_pred;

```

## Results and Analysis

### Filtro de Kalman

Se simuló el filtro de Kalman para un sistema con aceleración constante, y se obtuvieron gráficos de la posición y velocidad estimadas frente a las verdaderas.

![image](https://github.com/user-attachments/assets/7d43c385-b154-4b35-8d91-3e9eb4ff5e95)

### Filtro de Kalman Extendido

En la simulación del EKF, se utilizó un modelo no lineal con mediciones ruidosas de radar. Los resultados mostraron que el EKF fue capaz de rastrear la aeronave con precisión creciente a lo largo del tiempo.

![image](https://github.com/user-attachments/assets/9c6d915e-ac10-4e99-8e2b-1abc86e40321)

## Conclusions

### Filtro de Kalman

El Filtro de Kalman fue efectivo para sistemas lineales, mostrando que las estimaciones de posición y velocidad convergen a los valores verdaderos a medida que se actualizan con nuevas mediciones.

### Filtro de Kalman Extendido

El Filtro de Kalman Extendido demostró ser una herramienta poderosa para tratar sistemas no lineales, como las mediciones de radar. A medida que el filtro recibe más información, el error disminuye significativamente, lo que confirma la robustez del EKF para el seguimiento de la posición y velocidad de una aeronave con mediciones ruidosas.
