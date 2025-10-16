%% 
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = [1*-6e-06 1*9e-06 -0.00093378];
S.t_sim = 100;
S.omega_0 = [-0.01 0.00 0.06]';
S.EA_0 = deg2rad([-13 ,15, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';
omega_0_init = [0 0 0]';
q_0_init = [1 0 0 0]';
r_z_init = 2*S.r_0(3);
S.x_0_ukf = [omega_0_init; S.q_0; r_z_init];

Q_omega = 5e-5;
Q_q = 4e-5;
Q_r_z = 5e-13;

S.Q_ukf  = diag([Q_omega Q_omega Q_omega Q_q Q_q Q_q Q_q Q_r_z]);

out = simFromStruct('main_sim',S);

f = gen_single_fig();

t = out.r_z_UKF.time;
r_z_UKF = squeeze(out.r_z_UKF.signals.values);
sigma_UKF = squeeze(out.sigma_r_z.signals.values);

plot(t,r_z_UKF,'LineWidth',1.5)
hold on
plot(t,r_z_UKF+sigma_UKF,'--','Color',[0.000 0.447 0.741])
plot(t,r_z_UKF-sigma_UKF,'--','Color',[0.000 0.447 0.741])
grid on
fontsize(13,'points');
hold on
yline(S.r_0(3),'--','LineWidth',1.5)
xlabel("Time [s]")
ylabel("UKF Estimated $r_z$")

disp("Predicted: " + r_z_UKF(end));
disp("True: " + S.r_0(3))
disp("Error: " + (r_z_UKF(end) - S.r_0(3)))
disp("1 sigma: " + sigma_UKF(end))
disp("CV: " + (100*sigma_UKF(end)/abs(r_z_UKF(end))))
disp("Corresponding steps: " + r_z_UKF(end)*(S.m_s/S.mmu)*1e3*3200)

%%

pred_array = [-0.029066 -0.0096421 -0.00090163 -1.943e-05];
sigma_array = [0.000177 6.1488e-05 6.1592e-05 3.7934e-05];
err_array = [0.00093378 3.5791e-05 3.2145e-05 1.2715e-05];

figure
semilogy(linspace(1,4,4),pred_array,'--o')
hold on
semilogy(linspace(1,4,4),pred_array+sigma_array,'--o')
semilogy(linspace(1,4,4),pred_array-sigma_array,'--o')
%%
num_tests = 5;
r_z_array = zeros(num_tests,1);
cov_array = zeros(num_tests,1);
S.r_0 = [-6e-06 9e-06 -3*10^-2];
for j = 1:num_tests
    disp("Beginning test: " + j)
    out = simFromStruct('main_sim',S);

    r_z_UKF = squeeze(out.r_z_UKF.signals.values);
    sigma_UKF = squeeze(out.sigma_r_z.signals.values);

    r_z_array(j) = r_z_UKF(end);
    cov_array(j) = sigma_UKF(end);


    S.r_0(3) = S.r_0(3)-0.5*r_z_UKF(end);
    S.x_0_ukf = [omega_0_init; q_0_init; 1.5*r_z_UKF(end)];
end

figure
plot(linspace(1,5,5),r_z_array)