%% 4-state UKF used in the second phase of the hybrid balancing scheme for balancing the r z component.

clc
clear
data_path = 'May_27_UKF_4.mat';
load(data_path)
data_path = fullfile(pwd,data_path);
set_param('SADS_UKF/From File', 'FileName', data_path);  
t_final = size(output,1)*10;
sample_time = 0.1;
%%
% EKF Hyperparameters
n = 8;
alpha   = 1e-3;         % small, positive
kappa   = 0;            % usually 0
beta    = 2;            % optimal for Gaussian
lambda  = alpha^2*(n+kappa) - n;


% Test-bed characteristics 
m_s = 28.542; % [kg]
J = [1.301  0 0;
      0  1.023  0;
       0  0 1.253];
% r_0 = [0.1*10^-3, 0.1*10^-3 ,-1*10^-3]';
r_0 = [-0.003 0.006 -0.005]';
g_N = [0 0 -9.81];


% Filter matrices
R = diag([0.0005 0.0005 0.0005].^2);      % measurement-noise covariance
H = [eye(3)  zeros(3,5)];             % measurement matrix

% [omega, q, r_z]
Q_omega = 5e-5;
Q_q = 4e-11;
Q_r_z = 5e-13;

Q = diag([Q_omega Q_omega Q_omega Q_q Q_q Q_q Q_q Q_r_z]);



% setting initial conditions for the filter
omega_0 = output(5:7,1);
EA_0 = deg2rad(output(2:4,1));

% transformations are inertial to body
% XYZ rotation order must be explicitly specified
q_0 = eul2quat(EA_0',"XYZ")'; 
x_0 = [omega_0; q_0; -0.01];

n = size(x_0,1);
P_0 = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);

out = sim('SADS_UKF.slx');

disp(squeeze(out.r_z_est.signals.values(end)))

figure
subplot(2,1,1)
yline(0)
hold on
plot(out.r_z_est.time,squeeze(out.r_z_est.signals.values),'LineWidth',1.5)
grid on
xlabel('Time [s]')
title('$r_z$ Estimate')

subplot(2,1,2)
yline(0)
hold on
plot(out.sigma_r_z.time,squeeze(out.sigma_r_z.signals.values),'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('$r_z$ Covariance')