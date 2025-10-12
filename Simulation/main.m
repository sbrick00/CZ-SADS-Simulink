clc
clear

addpath("sim_helpers\")

t_sim = 800;

% SADS Values
m_s = 29; % [kg]
J_0 = [1.1  -0.014 0.004;
      -0.014  1.1  0.008;
       0.004  0.008 1.2];

r_0 = [9*10^-3, 2*10^-3 ,-3*10^-2]'; % solveable condition (displaced battery)
% r_0 = [0.1*10^-3, 0.1*10^-3 ,-1*10^-3]'; % simple rocking motion
% r_0 = [0, 0, -2*10^-3]'; % purely vertical
% r_0 = [0 0 0]'; % no inbalance

omega_0 = [-0.00 0.00 0.06]'; % slight spin, helps with stability
% omega_0 = [0 0.00 0.00]';

% EA_0 = deg2rad([1 0 45]'); 
EA_0 = deg2rad([-9 ,9, 0]');
q_0 = eul2quat(EA_0',"XYZ")'; 
q_d = [1 0 0 0]';
q_e_0 = quatmultiply(q_d',q_0');
q_e_0 = [q_e_0(2) q_e_0(3) q_e_0(4)]';

g_N = [0 0 -9.81];

% idea 1: higher gain scale, all gains similar
gainScale = 0.5;
K_omega = gainScale*0.6;
K_q = gainScale*0.1;
K_I = gainScale*2;

% for simulating plant ("truth")
process_variance = 1e-5; % variance of disturbance torques
% "sample time" for process noise, must be much shorter than the timescale of
% the system dynamics
T_s = 0.01; 
process_PSD = (process_variance^2)*T_s;

% Control System
sample_rate = 15; % IMU sample rate
sample_time = 1/sample_rate; % IMU sample period

% for simulating IMU
gyro_noise_power = 5.235988e-5; % [rad/s/sqrt(Hz)] provided on sensor datasheet
gyro_PSD = gyro_noise_power^2; % IMU power spectral density
gyro_bias_instability = (pi/180)*(6/3600); % [rad/s] provided on datasheet
% gyro_bias_instability = 0; % [rad/s] provided on datasheet

accel_noise_power = 9.8*(70e-6); % [m/s^2/sqrt(Hz)]
accel_PSD = accel_noise_power^2;
accel_bias_instability = 9.8*40e-6; % [(m/s)/s]

r_imu_b = [0.1524, 0, 0.1524];

% for UKF ---------------------------------------------------------
% R = diag([0.005 0.005 0.005].^2);      % measurement-noise covariance
% H = [eye(3)  zeros(3,5)];             % measurement matrix
% Q  = diag([5e-5 5e-5 5e-5 4e-11 4e-11 4e-11 4e-11 5e-13]);
% omega_0_init = [0 0 0]';
% q_0_init = [1 0 0 0]';
% x_0 = [omega_0_init; q_0_init; -0.02];
% n = size(x_0,1);
% P_0 = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);

% for EKF -----------------------------------------------------------
Q  = diag([5e-5 5e-5 5e-5 4e-11 4e-11 4e-11 4e-11]);
L = [sample_time*(J_0\eye(3)), zeros(3,4);
     zeros(4,3),               zeros(4,4)];
P_0 = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);
R = diag([0.005 0.005 0.005].^2);
x_0 = [0 0 0, 1 0 0 0]';

% for reaction wheels -----------------------------------------------
theta = deg2rad(63); % wheel pyramid elevation angle
a = cos(theta);
c = cos(theta);
b = sin(theta);
d = sin(theta);
A = [a -a  0  0;
     0  0  c -c;
     b  b  d  d]; % 4x1 wheel torques to 3x1 body torques

A_pinv = pinv(A); 
I_wheel = 1.698e-3; % kg*m^2
wheel_setpoint = [10.5 10.5 10.5 10.5]'; % ~ 100 RPM
A_truth = A;

% For FSFB Controller ------------------------------------------------
T_s = 8; % [s]
zeta = 0.8; 
w_n = 4/(zeta*T_s);
Kp_FSFB = 2.*(w_n^2).*diag(diag(J_0));
Kd_FSFB = 2.*zeta.*w_n.*diag(diag(J_0));

% For 3-axis Momentum Tracking -----------------------------------------
h_d_phase = [5 18 60];
h_d_amp = 10*[0.011 0.016 0.013]; % [Nms]
h_d_period = [4 4 4]; % [s]

Kappa = 5*diag([1 1 1]);
Gamma = 0.01*diag([0.005 0.005 0.09] );

mot_amp = 100*[0.0011 0.0016 0.0008 0.0013];
mot_period = [5 7 4 8];
mot_phase = [5 18 60 98];

% LEAST SQUARES REGRESSION METHOD ---------------------------------------

h_d_phase_LSR = [5 18 60];
h_d_amp_LSR = 2.2*[0.011 0.016 0.013]; % [Nms]
h_d_period_LSR = [12 9 14]; % [s]

Kappa_LSR = 0.05*diag([1 1 1]);

num_tests = 15;
r_err = zeros(num_tests,3);
J_true = [J_0(1,1),J_0(2,2),J_0(3,3),J_0(1,2),J_0(1,3),J_0(2,3)]';

%{ 
NOTES FOR LATER
WHEEL MOTOR + FLYWHEEL MASS = 0.774 kg, total mass is probably ~ 0.825 kg
WHEEL INERTIA = 0.00088247 kg * m^2
%}
%%

% BEGIN TEST LOOP
for j = 1:num_tests % ------------------------------------------------------
disp("Starting test " + j)
out = sim('main_sim.slx');

t = out.tout;
use_EKF = 0; % use EKF measurements or truth sim data
if use_EKF == 1
    g_b = squeeze(out.g_b_EKF.signals.values)';
    h_wheels = squeeze(out.h_wheels.signals.values)';
    omega_b = squeeze(out.omega_EKF.signals.values)';
    t = out.omega_EKF.time;
    N = numel(t);
    dt = [0; diff(t)];
else 
    LSR_sample_time = 0.1;
    omega_b = out.omega_b.signals.values;
    g_b = squeeze(out.g_b.signals.values)';
    tau_c = out.tau_truth.signals.values;
    h_wheels = squeeze(out.h_wheels.signals.values)';
    % rewrite signals to only sample every sample_time seconds
    t0 = t(1);
    tf = t(end);
    t_grid = (t0:LSR_sample_time:tf).';
    
    % Use 'previous' (ZOH) or 'linear' depending on what you want
    method = 'previous';  % or 'linear'
    
    omega_b  = interp1(out.omega_b.time, omega_b,  t_grid, method, 'extrap');
    g_b      = interp1(out.g_b.time, g_b,      t_grid, method, 'extrap');
    tau_c    = interp1(out.tau_truth.time, tau_c,    t_grid, method, 'extrap');
    h_wheels = interp1(out.h_wheels.time, h_wheels, t_grid, method, 'extrap');
    t        = t_grid;
    N = numel(t);
    dt = [0; diff(t)];
end

h_wheels = -h_wheels;



Omega_of_omega = @(w)[ ...
    w(1)  0     0     w(2)  w(3)   0   ;
    0     w(2)  0     w(1)  0      w(3);
    0     0     w(3)  0     w(1)   w(2)];

% Precompute per-sample Xi(ω), (ω×), (g×)
Omega  = zeros(3,6,N);
omega_cross = zeros(3,3,N);
g_cross  = zeros(3,3,N);
for k = 1:N
    w = omega_b(k,:).';
    g = g_b(k,:).';
    Omega(:,:,k)  = Omega_of_omega(w);
    omega_cross(:,:,k) = skew(w);
    g_cross(:,:,k)  = skew(g);
end

% Time integrals via cumulative trapezoid
% ∫ (ω×) Xi dt  and  ∫ (g×) dt   and  ∫ (ω× h) dt
Int_omega_cross_Omega = zeros(3,6,N);
Int_g_cross     = zeros(3,3,N);
Int_omega_cross_h  = zeros(3,1,N);

omega_cross_Omega_sum = zeros(3,6);
g_cross_sum     = zeros(3,3);
omega_cross_h  = zeros(3,1);

for k = 2:N
    h_mid   = 0.5*(h_wheels(k,:).'+h_wheels(k-1,:).');
    Omx_mid = 0.5*(omega_cross(:,:,k) + omega_cross(:,:,k-1));
    Xi_mid  = 0.5*(Omega(:,:,k)  + Omega(:,:,k-1));
    Gx_mid  = 0.5*(g_cross(:,:,k)  + g_cross(:,:,k-1));
    dtk     = t(k) - t(k-1);

    omega_cross_Omega_sum = omega_cross_Omega_sum + Omx_mid*Xi_mid * dtk;     % 3x6
    g_cross_sum     = g_cross_sum     + Gx_mid * dtk;             % 3x3
    omega_cross_h  = omega_cross_h  + Omx_mid * h_mid * dtk;    % 3x1

    Int_omega_cross_Omega(:,:,k) = omega_cross_Omega_sum;
    Int_g_cross(:,:,k)     = g_cross_sum;
    Int_omega_cross_h(:,:,k)  = omega_cross_h;
end

% Build stacked regression A*theta = b over samples k = 2..N
M = (N-1)*3;
A = zeros(M, 9);   % [ 3x6 for Jtilde | 3x3 for mr ]
b = zeros(M, 1);

Omega0 = Omega(:,:,1);
h0  = h_wheels(1,:).';

row = 1;
for k = 2:N
    % Left blocks:
    AJ = (Omega(:,:,k) - Omega0) + Int_omega_cross_Omega(:,:,k);   % 3x6
    Am = Int_g_cross(:,:,k);                            % 3x3

    % Right-hand side:
    bk = (h_wheels(k,:).' - h0) - Int_omega_cross_h(:,:,k); % 3x1

    % Stack
    A(row:row+2, :) = [AJ, Am];
    b(row:row+2)    = bk;
    row = row + 3;
end

theta = pinv(A)*b;  % 9x1

mr  = theta(7:9);
r_est = (mr./m_s);


r_err(j,:) = (r_0 - r_est)';
r_0 = r_0 - r_est;
end % END TEST LOOP -------------------------------------------------
%%
figure
plot(linspace(1,num_tests,num_tests),r_err,'--o')
grid on