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

% r_z_0 = -10e-4; % for run 2
r_z_0 = -0.0001; % for run 4
x_0 = [omega_0; q_0; r_z_0];

n = size(x_0,1);
P_0 = diag([1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6 1e-6]);

out = sim('SADS_UKF.slx');

disp(squeeze(out.r_z_est.signals.values(end)))
f = gen_side_by_side_fig();

yline(0)
hold on
plot(out.r_z_est.time,squeeze(out.r_z_est.signals.values),'LineWidth',1.8)
hold on
plot(out.r_z_est.time, squeeze(out.r_z_est.signals.values) + squeeze(out.sigma_r_z.signals.values),'--','Color',[0.000 0.447 0.741])
plot(out.r_z_est.time, squeeze(out.r_z_est.signals.values) - squeeze(out.sigma_r_z.signals.values),'--','Color',[0.000 0.447 0.741])
grid on
xlabel('Time [s]')
ylabel('Estimated $r_z$ [m]')
legend("","Estimated Value","$1\sigma$","Location","southeast")
fontsize(13,'points')
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\UKF_hardware_failure.pdf','ContentType','vector');        % axes only

r_z = squeeze(out.r_z_est.signals.values);
sigma = squeeze(out.sigma_r_z.signals.values);

disp("r_z end: " + r_z(end));
disp("sigma end: " + sigma(end));

%%
r_z_array = [-2.0928e-03, -6.3928e-04, -3.0928e-04,-1.0928e-04];
sigma_array = [1.0928e-04, 0.6e-04, -1.0928e-04,-2.0928e-04];

r_z_array_exp = [-2.0928e-03, -6.3928e-04, -3.0928e-04,-1.0928e-04];
sigma_array_exp = [1.0928e-04, 0.6e-04, -1.0928e-04,-2.0928e-04];

n = length(r_z_array);
f = gen_single_fig();
% plot(linspace(1,n,n),r_z_array,'--o','Color',[0.000 0.447 0.741],'LineWidth',1.5);
yline(0)
hold on
errorbar(linspace(1,n,n),r_z_array,sigma_array,'--o','Color',[0.000 0.447 0.741],'LineWidth',1.5)
xlabel("Iteration Number")
ylabel("Estimated $r_z\pm1\sigma$ [m]")
grid on
fontsize(13,'points')
xlim([0 n+1])
xticks(linspace(0,5,6))
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\UKF_hardware_iterations.pdf','ContentType','vector');        % axes only

