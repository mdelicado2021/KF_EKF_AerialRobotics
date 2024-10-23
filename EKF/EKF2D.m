clear all
close all
clc

%% Parámetros del problema
dt = 0.1; % Intervalo de tiempo
num_itr = 100; % Número de iteraciones

% Posición del radar
x_r = 0;
y_r = 0;

% Estado inicial del avión (real)
x0 = 0; 
y0 = 1000; 
v_real = 100; 

% Vector de estado inicial real [x; v; y]
x_real = [x0; v_real; y0];  % Estado real

% Estado inicial estimado (media inicial μ0)
mu = [100; 50; 500];  % Estado estimado inicial [x; v; y]

% Covarianza inicial Σ0
P = diag([100, 25, 100]);  % Covarianza inicial para x, v, y

% Ruido del proceso (avión: velocidad y altitud)
Q = diag([0, 0.02^2, 0.1^2]);

% Ruido de las mediciones (radar: rango y ángulo)
R = diag([5^2, (0.5*pi/180)^2]); % Convertir ángulo a radianes

% Matriz de transición del estado (solo cambia x=x+v⋅dt, 'v' e 'y' son ctes)
F = [1 dt 0;
     0  1 0;
     0  0 1];

% Medidas del radar en función del estado
h = @(x) [sqrt((x(1) - x_r)^2 + (x(3) - y_r)^2);  % rango
          atan2(x(3) - y_r, x(1) - x_r)];         % ángulo

%% Simulación
mu_hist = zeros(3, num_itr); % Para guardar la historia del estado estimado
P_hist = zeros(3, 3, num_itr); % Para guardar la historia de la covarianza
x_real_hist = zeros(3, num_itr); % Para guardar la historia del estado real
innovation_hist = zeros(2, num_itr); % Para guardar la historia de la innovación

for k = 1:num_itr
    % Simulación del movimiento real del avión (sin ruido)
    x_real = F * x_real; % Actualización del estado real
    
    % Simulación de las mediciones con ruido
    z_real = h(x_real) + sqrt(R) * randn(2, 1);
    
    % Predicción del estado y la covarianza (para el estado estimado)
    mu_pred = F * mu;
    P_pred = F * P * F' + Q;
    
    % Jacobiano de la función de medición
    H = [(mu_pred(1) - x_r)/sqrt((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0, (mu_pred(3) - y_r)/sqrt((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2);
         -(mu_pred(3) - y_r)/((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0, (mu_pred(1) - x_r)/((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2)];
     
    % Innovación (diferencia entre la medición real y la predicha)
    y_k = z_real - h(mu_pred);
    
    % Covarianza de la innovación
    S = H * P_pred * H' + R;
    
    % Ganancia de Kalman
    K = P_pred * H' / S;
    
    % Actualización del estado estimado y la covarianza
    mu = mu_pred + K * y_k;
    P = (eye(3) - K * H) * P_pred;
    
    % Guardar la historia
    mu_hist(:, k) = mu;
    P_hist(:, :, k) = P;
    x_real_hist(:, k) = x_real;
    innovation_hist(:, k) = y_k;  % Guardar la innovación
end

%% Gráficas de resultados
t = (0:num_itr-1) * dt;

% Gráfica de la posición 'x' y error
figure;
subplot(2,1,1);
plot(t, x_real_hist(1,:), t, mu_hist(1,:));
xlabel('Tiempo [s]');
ylabel('Posición x [m]');
legend('Xreal', 'Xestimada');

subplot(2,1,2);
semilogy(t, abs(x_real_hist(1,:) - mu_hist(1,:)));
xlabel('Tiempo [s]');
ylabel('Error absoluto (Posición x)');

% Gráfica de la velocidad y error
figure;
subplot(2,1,1);
plot(t, x_real_hist(2,:), t, mu_hist(2,:));
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
legend('Velocidad real', 'Velocidad estimada');

subplot(2,1,2);
semilogy(t, abs(x_real_hist(2,:) - mu_hist(2,:)));
xlabel('Tiempo [s]');
ylabel('Error absoluto (Velocidad)');

% Gráfica de la posición 'y' y error
figure;
subplot(2,1,1);
plot(t, x_real_hist(3,:), t, mu_hist(3,:));
xlabel('Tiempo [s]');
ylabel('Altitud [m]');
legend('Altitud real', 'Altitud estimada');

subplot(2,1,2);
semilogy(t, abs(x_real_hist(3,:) - mu_hist(3,:)));
xlabel('Tiempo [s]');
ylabel('Error absoluto (Altitud)');

% Gráfica de la innovación
figure;
plot(t, innovation_hist(1,:), t, innovation_hist(2,:));
xlabel('Tiempo [s]');
ylabel('Innovación');
legend('Innovación en rango', 'Innovación en ángulo');

% Gráfica de la varianza de la matriz de covarianza
figure;
plot(t, squeeze(P_hist(1,1,:)), t, squeeze(P_hist(2,2,:)), t, squeeze(P_hist(3,3,:)));
xlabel('Tiempo [s]');
ylabel('Varianza');
legend('Varianza en x', 'Varianza en v', 'Varianza en y');

