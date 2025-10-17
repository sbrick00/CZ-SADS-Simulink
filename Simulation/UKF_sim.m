%% 
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = [1*-6e-06 1*9e-06 -2e-9];
S.t_sim = 100;
S.omega_0 = [-0.01 0.04 0.15]';
S.EA_0 = deg2rad([-18 ,21, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';
omega_0_init = [0 0 0]';
q_0_init = [1 0 0 0]';
r_z_init = 3*S.r_0(3);
S.x_0_ukf = [omega_0_init; S.q_0; r_z_init];

Q_omega = 5e-5;
Q_q = 4e-6;
Q_r_z = 5e-13;

S.Q_ukf  = diag([Q_omega Q_omega Q_omega Q_q Q_q Q_q Q_q Q_r_z]);

out = simFromStruct('main_sim',S);

f = gen_single_fig();

start_idx = 3;
t = out.r_z_est.time(start_idx:end);
r_z_UKF = squeeze(out.r_z_est.signals.values(start_idx:end));
sigma_UKF = squeeze(out.sigma_r_z.signals.values(start_idx:end));

plot(t,r_z_UKF,'LineWidth',1.5)
hold on
plot(t,r_z_UKF+sigma_UKF,'--','Color',[0.000 0.447 0.741])
plot(t,r_z_UKF-sigma_UKF,'--','Color',[0.000 0.447 0.741])
grid on
fontsize(13,'points');
hold on
yline(S.r_0(3),'--','LineWidth',1.5)
xlabel("Time [s]")
ylabel("Estimated $r_z$ [m]")

legend("Estimated Value","$1\sigma$","","True Value")

disp("Predicted: " + r_z_UKF(end));
disp("True: " + S.r_0(3))
disp("Error: " + (r_z_UKF(end) - S.r_0(3)))
disp("1 sigma: " + sigma_UKF(end))
disp("CV: " + (100*sigma_UKF(end)/abs(r_z_UKF(end))))
disp("Corresponding steps: " + r_z_UKF(end)*(S.m_s/S.mmu)*1e3*3200)

%%
r_z_array_exp = [-2.0928e-03, -6.3928e-04, -3.0928e-04,-1.0928e-04];
sigma_array_exp = [1.0928e-04, 0.6e-04, -1.0928e-04,-1.5928e-04];

r_z_array = [-2.0928e-03, -9.4876e-05, -1.4876e-05,-9.4876e-65];
sigma_array = [6.0928e-05, 4.4744e-05, 2.4744e-05, 2.4744e-05];

n = length(r_z_array);
f = gen_side_by_side_fig();
% plot(linspace(1,n,n),r_z_array,'--o','Color',[0.000 0.447 0.741],'LineWidth',1.5);
yline(0)
hold on
errorbar(linspace(1,n,n),r_z_array_exp,sigma_array_exp,'--o','LineWidth',1.5)
errorbar(linspace(1,n,n),r_z_array,sigma_array,'--o','LineWidth',1.5)
legend("","Experimental","Simulation","Location","southeast")
xlabel("Iteration Number")
ylabel("Estimated $r_z\pm1\sigma$ [m]")
grid on
fontsize(13,'points')
xlim([0 n+1])
xticks(linspace(0,5,6))
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\UKF_comparison.pdf','ContentType','vector');        % axes only