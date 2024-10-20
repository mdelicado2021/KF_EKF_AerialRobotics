%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% 2022-11, Madrid, X. Chen
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Kalman Filter, straight line with constant acceleration control input
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------

function [mu, cov, z_bar, inn, z, x_real]= KF_straight...
    (dt, num_itr, parm_OB, parm_RW)
%-------------------------------------------------------------------------- 
% Kalman Filter, straight line with constant acceleration control input
%-------------------------------------------------------------------------- 
%---------------------------------Input------------------------------------
%-------------------------------------------------------------------------- 
% ---> dt [s] 
%      [1:1], time duration at each step
% ---> num_itr 
%      [1:1], number of iterations
% ---> parm_RW.p_0 [m] 
%      [1:1], initial position
% ---> parm_RW.v_0 [m/s] 
%      [1:1], initial velocity
% ---> parm_RW.u_cmd [m/s^2]
%      [1:1], commanded constant acceleration 
% ---> parm_RW.delta_acc [m/s^2]
%      [1:1], standard deviation of accelerometer noise
% ---> parm_RW.delta_range [m]
%      [1:1], standard deviation of range measurement noise
% ---> parm_OB.delta_acc [m/s^2]
%      [1:1], standard deviation of accelerometer noise
% ---> parm_OB.delta_range [m]
%      [1:1], standard deviation of range measurement noise
% ---> parm_OB.mu_0 [m; m/s]
%      [2:1], initial state estimate
% ---> parm_OB.cov_0 [m^2 m^2/s; m^2/s m^2/s^2]
%      [2:2], initial error covariance estimate
%-------------------------------------------------------------------------- 
%---------------------------------Output-----------------------------------
%--------------------------------------------------------------------------
% ---> mu [m; m/s]
%      [2:num_itr+1], state estimation from KF at each time step
% ---> cov [m^2 m^2/s; m^2/s m^2/s^2]
%      [2:2:num_itr+1], covariance matrix from KF at each time step
% ---> z_bar [m]
%      [1:num_itr+1], range measurement predicted by KF
% ---> inn [m]
%      [1:num_itr+1], innovation vector in KF at each time step
% ---> z [m]
%      [1:num_itr+1], range measurement simulated in the real world
% ---> x_real [m; m/s]
%      [2:num_itr+1], state simulated in the real world
%-------------------------------------------------------------------------- 
 
[x_real, u, z] = mod_RW(dt, num_itr, parm_RW);
[mu, cov, z_bar, inn] = mod_OB(dt, num_itr, parm_OB, u, z);

end
%%
function [x_real, u, z] = mod_RW(dt, num_itr, parm_RW)
%-------------------------------------------------------------------------- 
% Real world simulation module
%-------------------------------------------------------------------------- 
%---------------------------------Input------------------------------------
%-------------------------------------------------------------------------- 
% ---> dt [s] 
%      [1:1], time duration at each step
% ---> num_itr 
%      [1:1], number of iterations
% ---> parm_RW.p_0 [m] 
%      [1:1], initial position
% ---> parm_RW.v_0 [m/s] 
%      [1:1], initial velocity
% ---> parm_RW.u_cmd [m/s^2]
%      [1:1], commanded constant acceleration 
% ---> parm_RW.delta_acc [m/s^2]
%      [1:1], standard deviation of accelerometer noise
% ---> parm_RW.delta_range [m]
%      [1:1], standard deviation of range measurement noise 
%-------------------------------------------------------------------------- 
%---------------------------------Output-----------------------------------
%-------------------------------------------------------------------------- 
% ---> x_real [m; m/s]
%      [2:num_itr+1], simulated state 
% ---> u [m]
%      [1:num_itr+1], simulated accelerameter measurement 
% ---> z [m]
%      [1:num_itr+1], simulated range measurement 
%-------------------------------------------------------------------------- 
x_real = zeros(2, num_itr+1);
% [2:num_itr+1], each column is the state vector    
% The initial state at t=0 is [0,0] in real world; 
z = zeros(1, num_itr+1);
% [num_itr+1:1], each column is the range finder measurement p_range.
% This will be used in the KF algorithm as sensor measurement input
u = zeros(1, num_itr+1);
% [num_itr+1:1], each column is the accelerameter measurement a_acc.
% This will be used in the KF algorithm as sensor control input



%% Linear state model (A amd B) and observation model (C)

A = [1 dt; 0 1];      % Matriz de transición de estado
B = [0.5*dt^2; dt];   % Matriz de control (efecto de la aceleración en posición y velocidad)

% Matriz de observación del sensor de rango (solo observa la posición)
H = [1 0];            % Solo la posición se mide con el sensor de rango



%% Initial condition

% Inicialización de la posición y velocidad
x_real(:, 1) = [parm_RW.p_0; parm_RW.v_0];  % Estado inicial [posición inicial; velocidad inicial]

% Inicialización de la medición del acelerómetro y el sensor de rango
u(1) = parm_RW.u_cmd + parm_RW.sigma_acc * randn();  % Aceleración inicial con ruido
z(1) = H * x_real(:, 1) + parm_RW.sigma_range * randn();  % Medición de rango inicial con ruido



%% Simulated state (Euler method) and acceleromete/range measurements
i=2;
while i<= num_itr+1

    % Actualización del estado real del sistema (mundo real) con la aceleración constante
    x_real(:, i) = A * x_real(:, i-1) + B * parm_RW.u_cmd;
    
    % Medición ruidosa del acelerómetro (aceleración medida)
    u(i) = parm_RW.u_cmd + parm_RW.sigma_acc * randn();
    
    % Medición ruidosa del sensor de rango (posición medida)
    z(i) = H * x_real(:, i) + parm_RW.sigma_range * randn();

    i=i+1;
end
 
end
%%
function [mu, cov, z_bar, inn] = mod_OB(dt, num_itr, parm_OB, u, z)
%-------------------------------------------------------------------------- 
% Onboard software module
%-------------------------------------------------------------------------- 
%---------------------------------Input------------------------------------
%-------------------------------------------------------------------------- 
% ---> dt [s] 
%      [1:1], time duration at each step
% ---> num_itr 
%      [1:1], number of iterations 
% ---> parm_OB.delta_acc [m/s^2]
%      [1:1], standard deviation of accelerometer noise
% ---> parm_OB.delta_range [m]
%      [1:1], standard deviation of range measurement noise
% ---> parm_OB.mu_0 [m; m/s]
%      [2:1], initial guess of position and velocity
% ---> parm_OB.cov_0 [m^2 m^2/s; m^2/s m^2/s^2]
%      [2:2], initial covariance matrix
%-------------------------------------------------------------------------- 
%---------------------------------Output-----------------------------------
%--------------------------------------------------------------------------
% ---> mu [m; m/s]
%      [2:num_itr+1], state estimation from KF at each time step
% ---> cov [m^2 m^2/s; m^2/s m^2/s^2]
%      [2:2:num_itr+1], covariance matrix from KF at each time step
% ---> z_bar [m]
%      [1:num_itr+1], range measurement simulated in the real world
% ---> inn [m]
%      [1:num_itr+1], innovation vector in KF at each time step 
%-------------------------------------------------------------------------- 
 
mu_bar  = zeros(2, num_itr+1);
% [2:num_itr+1], each column is the predicted state estimate
mu      = zeros(2, num_itr+1);
% [2:num_itr+1], each column is the updated state estimate
cov_bar = zeros(2, 2, num_itr+1);
% [2:2:num_itr+1], each page the the predicted error covariance 
cov     = zeros(2, 2, num_itr+1);
% [2:2:num_itr+1], each page the the updated error covariance 
z_bar   =  zeros(1, num_itr+1);
% [1:num_itr+1], each column is the predicted measurement.
inn     =  zeros(1, num_itr+1);
% [1:num_itr+1], each column is the innovation.



%% KF state model and observation model

% Matrices de estado (A y B) y modelo de observación (H)
A = [1 dt; 0 1];       % Matriz de transición de estado
B = [0.5*dt^2; dt];    % Matriz de control
H = [1 0];             % Observación (solo se mide la posición)

% Covarianzas del proceso y del sensor
Q = [parm_OB.sigma_acc^2, 0; 0, parm_OB.sigma_acc^2]; % Covarianza del ruido del proceso (acelerómetro)
R = parm_OB.sigma_range^2;                            % Covarianza del ruido del sensor (rango)



%% Initial estimate and error covariance

mu(:, 1) = parm_OB.mu_0;  % Estimación inicial de estado
cov(:, :, 1) = parm_OB.cov_0;  % Matriz de covarianza inicial



%% Recursive Kalman Filter Algorithm
i=2;
while i<= num_itr+1

    %--- Predicción ---
    % Predicción del estado siguiente
    mu_bar(:, i) = A * mu(:, i-1) + B * u(i-1);  % Predicción del estado usando el control (aceleración)
    
    % Predicción de la covarianza de error
    cov_bar(:, :, i) = A * cov(:, :, i-1) * A' + Q;  % Predicción de la covarianza
    
    %--- Innovación (residual) ---
    z_bar(i) = H * mu_bar(:, i);  % Predicción de la medición de rango
    inn(i) = z(i) - z_bar(i);     % Innovación: diferencia entre la medición real y la predicha
    
    %--- Actualización ---
    % Ganancia de Kalman
    S = H * cov_bar(:, :, i) * H' + R;  % Varianza residual (S)
    K = cov_bar(:, :, i) * H' / S;      % Ganancia de Kalman
    
    % Actualización del estado
    mu(:, i) = mu_bar(:, i) + K * inn(i);  % Estado corregido
    
    % Actualización de la covarianza del error
    cov(:, :, i) = (eye(2) - K * H) * cov_bar(:, :, i);  % Covarianza corregida
     
    i=i+1;
end
 
end