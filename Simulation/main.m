clc
clear

% SADS Values
m_s = 29; % [kg]
J_0 = [1.815  -0.014 0.004;
      -0.014  1.348  0.008;
       0.004  0.008 1.475];

% r_0 = [4*10^-3, 2*10^-3 ,-2*10^-2]'; % solveable condition (displaced battery)
% r_0 = [0.1*10^-3, 0.1*10^-3 ,-1*10^-3]'; % simple rocking motion
r_0 = [0, 0, -2*10^-3]'; % purely vertical
% r_0 = [0 0 0]'; % no inbalance

omega_0 = [0 0.00 0.06]'; % slight spin, helps with stability
% omega_0 = [0 0.00 0.00]';

% EA_0 = deg2rad([1 0 45]'); 
EA_0 = deg2rad([-8, 15, 0]');
q_0 = eul2quat(EA_0',"XYZ")'; 
q_d = [1 0 0 0]';
q_e_0 = quatmultiply(q_d',q_0');
q_e_0 = [q_e_0(2) q_e_0(3) q_e_0(4)]';

g_N = [0 0 -9.81];


% idea 1: higher gain scale, all gains similar
gainScale = 0.5;
K_omega = gainScale*1;
K_q = gainScale*1;
K_I = gainScale*5;

% for simulating plant ("truth")
process_variance = 1e-5; % variance of disturbance torques
% "sample time" for process noise, must be much shorter than the timescale of
% the system dynamics
T_s = 0.01; 
process_PSD = (process_variance^2)*T_s;

% for simulating IMU
noise_power = 5.235988e-5; % [rad/s/sqrt(Hz)] provided on sensor datasheet
sensor_PSD = noise_power^2; % IMU power spectral density
sample_rate = 10; % IMU sample rate
sample_time = 1/sample_rate; % IMU sample period
bias_stability = (pi/180)*(6/3600);

% for UKF
% Filter matrices
R = diag([0.005 0.005 0.005].^2);      % measurement-noise covariance
H = [eye(3)  zeros(3,5)];             % measurement matrix
Q  = diag([5e-5 5e-5 5e-5 4e-11 4e-11 4e-11 4e-11 5e-13]);
omega_0_init = [0 0 0]';
q_0_init = [1 0 0 0]';
x_0 = [omega_0_init; q_0_init; -0.001];
n = size(x_0,1);
P_0 = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);


t_sim = 100;
%%
out = sim('main.slx');