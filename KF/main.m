%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% 2022-11, Madrid, X. Chen
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Kalman Filter, straight line with constant acceleration control input
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------


clear all
close all
%% Set Graphical Properties
set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); 
set(groot, 'defaultAxesFontSize', 11); 
set(groot, 'defaultAxesGridAlpha', 0.3); 
set(groot, 'defaultAxesLineWidth', 0.75);
set(groot, 'defaultAxesXMinorTick', 'on');
set(groot, 'defaultAxesYMinorTick', 'on');
set(groot, 'defaultFigureRenderer', 'painters');
set(groot, 'defaultLegendBox', 'off');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultLegendLocation', 'best');
set(groot, 'defaultLineLineWidth', 1); 
set(groot, 'defaultLineMarkerSize', 3);
set(groot, 'defaultTextInterpreter','latex');

dt          = 0.1; % [s]: time duration at each step 
num_itr     = 200; % [-]: number of iterations

%% Parameters for RealWorld software module 
parm_RW.p_0         = 1; % [m]: initial position
parm_RW.v_0         = 1; % [m/s]: initial velocity 
parm_RW.u_cmd       = 1; % [m/s^2]: commanded acceleration control input  
parm_RW.sigma_acc   = 1; % [m/s^2]: standard deviation of acceleration noise
parm_RW.sigma_range = 1; % [m]: standard deviation of range measurement noise

%% Parameters for OnBoard simulation module 
parm_OB.sigma_acc   = 1; % [m/s^2]: standard deviation of acceleration noise
parm_OB.sigma_range = 1; % [m]: standard deviation of range measurement noise
parm_OB.mu_0        = [10;10]; % [m; m/s]: initial state estimate 
parm_OB.cov_0       = [1 0; 0 1]; %[m^2 m^2/s; m^2/s m^2/s^2]: initial covariance matrix 

%% Call KF
[mu, cov, z_bar, inn, z, x_real]= KF_straight(dt, num_itr, parm_OB, parm_RW);
 
%% Plot results
t_arr   = linspace(0, dt*num_itr, num_itr+1); 
% [num_itr+1:1], discrete time array
 
set(gcf, 'Position', [500 300 350 500]); 
subplot(2,1,1); 
plot(t_arr, z, '.', t_arr, mu(1,:),  t_arr, x_real(1,:) )
title('Position')
xlabel('$ t \, [s]$'); 
ylabel('$ p \, [m]$');
legend('$p_{laser}$', '$p_{\mu}$', '$p_{real}$')
subplot(2,1,2);   
plot(t_arr,inn , '.' , t_arr, mu(1,:)-x_real(1,:), ...
    t_arr, sqrt(squeeze(cov(1,1,:))), ...
    t_arr, z-x_real(1,:))
xlabel('$ t \, [s]$');  
ylabel('error $[m]$');
legend('Innovation', '$p_{\mu}-p_{real}$','$\sqrt{\Sigma_{11}}$', '$p_{laser}-p_{real}$')
set(gcf, 'PaperPositionMode','auto')
print(gcf,'-painters','-depsc','kalman_eg_p'); 


figure
set(gcf, 'Position', [1000 300 350 500]); 
subplot(2,1,1); 
plot(t_arr, mu(2,:), t_arr, x_real(2,:))
title('Velocity')
xlabel('$ t \, [s]$');
ylabel('$ v \, [m/s]$');
legend('$v_{\mu}$', '$v_{real}$')
subplot(2,1,2);  
plot(t_arr, mu(2,:)-x_real(2,:), t_arr, sqrt(squeeze(cov(2,2,:))))
xlabel('$ t \, [s]$');  
ylabel('error $[m/s]$');
legend('$v_{\mu}-v_{real}$','$\sqrt{\Sigma_{22}}$')
set(gcf, 'PaperPositionMode','auto')
print(gcf,'-painters','-depsc','kalman_eg_v');