function S = setup()

addpath("sim_helpers\")
MY = [ ...
  0.000 0.447 0.741;  % blue
  0.850 0.325 0.098;  % orange
  0.929 0.694 0.125;  % yellow
  0.494 0.184 0.556;  % purple
  0.466 0.674 0.188;  % green
  0.301 0.745 0.933;  % cyan
  0.635 0.078 0.184]; % red

set(groot, 'defaultAxesColorOrder', MY);
% Optional: also fix default line styles if you want more than 7 curves:
set(groot, 'defaultAxesLineStyleOrder', {'-','--',':','-.'});


S.t_sim = 400;

% SADS Values
S.m_s = 29; % [kg]
S.J_0 = [1.298  -0.014 0.004;
      -0.014  1.025  0.008;
       0.004  0.008 1.243];
S.mmu = 2*((0.015*5) + (6*.193) + 0.177);


S.r_0 = [3*10^-3, 2*10^-3 ,-3*10^-2]'; % solveable condition (displaced battery)
% r_0 = [0.1*10^-3, 0.1*10^-3 ,-1*10^-3]'; % simple rocking motion
% r_0 = [0, 0, -2*10^-3]'; % purely vertical
% r_0 = [0 0 0]'; % no inbalance

S.omega_0 = [-0.00 0.00 0.00]'; % slight spin, helps with stability
% omega_0 = [0 0.00 0.00]';

% EA_0 = deg2rad([1 0 45]'); 
S.EA_0 = deg2rad([0 ,0, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")'; 
S.q_d = [1 0 0 0]';
q_e_0 = quatmultiply(S.q_d',S.q_0');
S.q_e_0 = [q_e_0(2) q_e_0(3) q_e_0(4)]';

S.g_N = [0 0 -9.81];

% Underactuated PID gains ----------------------------------------------
gainScale = 0.5;
S.K_omega = gainScale*0.6;
S.K_q = gainScale*0.1;
S.K_I = gainScale*2;

% for simulating plant ("truth") ----------------------------------------
process_variance = 1e-5; % variance of disturbance torques
% "sample time" for process noise, must be much shorter than the timescale of
% the system dynamics
T_s = 0.01; 
S.process_PSD = (process_variance^2)*T_s;

% Control System ---------------------------------------------------------
S.sample_rate = 15; % IMU sample rate
S.sample_time = 1/S.sample_rate; % IMU sample period
S.tau_delay = 1e-5;

% for simulating IMU --------------------------------------------
gyro_noise_power = 5.235988e-5; % [rad/s/sqrt(Hz)] provided on sensor datasheet
S.gyro_PSD = gyro_noise_power^2; % IMU power spectral density
S.gyro_bias_instability = (pi/180)*(6/3600); % [rad/s] provided on datasheet
% gyro_bias_instability = 0; % [rad/s] provided on datasheet

accel_noise_power = 9.8*(70e-6); % [m/s^2/sqrt(Hz)]
S.accel_PSD = accel_noise_power^2;
S.accel_bias_instability = 9.8*40e-6; % [(m/s)/s]

S.r_imu_b = [0.1524, 0, 0.1524];

% for UKF ---------------------------------------------------------
S.R_ukf = diag([0.005 0.005 0.005].^2);      % measurement-noise covariance
S.H_ukf = [eye(3)  zeros(3,5)];             % measurement matrix
S.Q_ukf  = diag([5e-5 5e-5 5e-5 4e-11 4e-11 4e-11 4e-11 5e-13]);
omega_0_init = [0 0 0]';
q_0_init = [1 0 0 0]';
S.x_0_ukf = [omega_0_init; q_0_init; -0.02];
S.n_ukf = size(S.x_0_ukf,1);
S.P_0_ukf = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);

% for EKF -----------------------------------------------------------
S.Q  = diag([5e-5 5e-5 5e-5 4e-11 4e-11 4e-11 4e-11]);
S.L = [S.sample_time*(S.J_0\eye(3)), zeros(3,4);
       zeros(4,3)                , zeros(4,4)];
S.P_0 = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);
S.R = diag([0.005 0.005 0.005].^2);
S.x_0 = [0 0 0, 1 0 0 0]';

% for reaction wheels -----------------------------------------------
theta = deg2rad(27);
c = cos(theta)/sqrt(2);
s = sin(theta);
A = [ c  -c  -c   c;
      c   c  -c  -c;
      s   s   s   s ];

S.A_pinv = pinv(A); 
S.I_wheel = 1.698e-3; % kg*m^2
S.wheel_setpoint = [10.5 10.5 10.5 10.5]'; % ~ 100 RPM
S.A_truth = zeros(3,4); % inject mounting error into truth model
alpha = deg2rad(2);    % ~1 degree error tilt

for i = 1:4
    ei = A(:,i);                % nominal unit axis
    B = null(ei.');             % 3x2 basis of plane perpendicular to ei
    phi = 2*pi*rand;            % random in-plane direction
    tdir = B*[cos(phi); sin(phi)]; % unit vector in ei^⊥
    S.A_truth(:,i) = cos(alpha)*ei + sin(alpha)*tdir;
    S.A_truth(:,i) = S.A_truth(:,i) / norm(S.A_truth(:,i));
end

% For FSFB Controller ------------------------------------------------
T_s = 8; % [s]
zeta = 0.8; 
w_n = 4/(zeta*T_s);
S.Kp_FSFB = 2.*(w_n^2).*diag(diag(S.J_0));
S.Kd_FSFB = 2.*zeta.*w_n.*diag(diag(S.J_0));

% For 3-axis Momentum Tracking -----------------------------------------
S.h_d_phase = [0 0 0];
S.h_d_amp = 1.1.*[0.011 0.016 0.013]; % [Nms]
S.h_d_period = [21 19 30]; % [s]

S.Kappa = 5*diag([1 1 1]);
S.Gamma = 0.01*diag([0.005 0.005 0.09] );

% LEAST SQUARES REGRESSION METHOD ---------------------------------------

S.h_d_phase_LSR = [0 0 0];
S.h_d_amp_LSR =1.1*[0.011 0.016 0.013]; % [Nms]
S.h_d_period_LSR = [12 9 14]; % [s]

S.Kappa_LSR = 0.7*diag([1 1 1]);
%{ 
NOTES FOR LATER
WHEEL MOTOR + FLYWHEEL MASS = 0.774 kg, total mass is probably ~ 0.825 kg
WHEEL INERTIA = 0.00088247 kg * m^2
%}


end