clear all
close all
clc

%% Parámetros del problema
dt = 0.1; % Intervalo de tiempo
num_itr = 100; % Número de iteraciones

% Posición del radar
x_r = 0;
y_r = 0;

% Estado inicial del avión
x0 = 0; 
y0 = 1000; 

% Velocidad constante del avión
v = 100; 

% Vector de estado inicial [x; vx; y; vy]
x = [x0; v; y0; 0];

% Covarianza inicial
P = diag([100, 25, 500, 100]);

% Ruido del proceso (movimiento del avión)
Q = diag([0.02^2, 0.02^2, 0.1^2, 0.1^2]);

% Ruido de las mediciones (rango y ángulo)
R = diag([5^2, (0.5*pi/180)^2]); % Convertir ángulo a radianes

% Matriz de transición del estado (movimiento constante)
F = [1 dt 0  0;
     0  1 0  0;
     0  0 1 dt;
     0  0 0  1];

% Entrada de control (sin aceleración)
B = [0; 0; 0; 0];

% Medidas del radar en función del estado
h = @(x) [sqrt((x(1) - x_r)^2 + (x(3) - y_r)^2);
          atan2(x(3) - y_r, x(1) - x_r)];

%% Simulación
mu = x; % Inicialización del estado estimado
mu_hist = zeros(4, num_itr); % Para guardar la historia del estado estimado
P_hist = zeros(4, 4, num_itr); % Para guardar la historia de la covarianza
z_hist = zeros(2, num_itr); % Historia de las mediciones

for k = 1:num_itr
    % Simulación del movimiento real del avión
    x_real = F * x + B; % Actualización del estado real
    
    % Simulación de las mediciones con ruido
    z_real = h(x_real) + sqrt(R) * randn(2, 1);
    
    % Predicción del estado y la covarianza
    mu_pred = F * mu;
    P_pred = F * P * F' + Q;
    
    % Jacobiano de la función de medición
    H = [(mu_pred(1) - x_r)/sqrt((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0, (mu_pred(3) - y_r)/sqrt((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0;
         -(mu_pred(3) - y_r)/((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0, (mu_pred(1) - x_r)/((mu_pred(1) - x_r)^2 + (mu_pred(3) - y_r)^2), 0];
     
    % Innovación
    y_k = z_real - h(mu_pred);
    
    % Covarianza de la innovación
    S = H * P_pred * H' + R;
    
    % Ganancia de Kalman
    K = P_pred * H' / S;
    
    % Actualización del estado y la covarianza
    mu = mu_pred + K * y_k;
    P = (eye(4) - K * H) * P_pred;
    
    % Guardar la historia
    mu_hist(:, k) = mu;
    P_hist(:, :, k) = P;
    z_hist(:, k) = z_real;
    
    % Actualización del estado real
    x = x_real;
end

%% Gráficas de resultados
t = (0:num_itr-1) * dt;

% Gráfica de la posición y error
figure;
subplot(2,1,1);
plot(t, mu_hist(1,:), t, z_hist(1,:));
xlabel('Tiempo [s]');
ylabel('Posición horizontal [m]');
legend('Posición estimada', 'Posición real');

subplot(2,1,2);
semilogy(t, abs(mu_hist(1,:) - z_hist(1,:)));
xlabel('Tiempo [s]');
ylabel('Error absoluto (Posición)');

% Gráfica de la velocidad
figure;
subplot(2,1,1);
plot(t, mu_hist(2,:), t, z_hist(2,:));
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
legend('Velocidad estimada', 'Velocidad real');

subplot(2,1,2);
semilogy(t, abs(mu_hist(2,:) - z_hist(2,:)));
xlabel('Tiempo [s]');
ylabel('Error absoluto (Velocidad)');
