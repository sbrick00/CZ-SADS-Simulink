%%
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values 
S.r_0 = 1e-1.*[5*10^-3,-6*10^-3 ,-3*10^-2]';
S.omega_0 = 0.01.*[0.8 -0.9 1.3]';
S.EA_0 = deg2rad([-5 ,-6, 2]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';
S.t_sim = 80;
S.k_p = 0.09;
S.gamma = 0.0005;
S.r_hat_0 = [0 0 -0.05]';
S.tau_c_lim = 2;
S.K_q = 2;
out = simFromStruct('main_sim',S);

EA_plt = deg2rad(squeeze(out.EA_degs.signals.values));
omega_p_plt = squeeze(out.omega_p.signals.values);
del_r_plt = squeeze(out.del_r.signals.values);
t_plt = out.tout;

del_d_plt = (del_r_plt)*(S.m_s/S.mmu);

del_d_sol_x = -S.r_0(1)*(S.m_s/S.mmu);
del_d_sol_y = -S.r_0(2)*(S.m_s/S.mmu);

f = gen_tiled_layout(3);
t = tiledlayout(f,3,1,'Padding','compact','TileSpacing','compact');
ax1 = nexttile;

plot(out.tout,EA_plt(:,1:2)-EA_plt(end,1:2),'LineWidth',1.5)
grid on
ylabel("Euler Angles [rad]")
legend("$\phi$","$\theta$")

ax2 = nexttile;
plot(out.tout,omega_p_plt(:,1:3),'LineWidth',1.5)
grid on
ylabel("Projected Body Rates [rad/s]")
legend("$\omega_{p,x}$","$\omega_{p,x}$","$\omega_{p,z}$")

ax3 = nexttile;
plot(out.tout,del_d_plt(:,1:2),'LineWidth',1.5)
grid on 
ylabel("Sliding Mass Positions [m]")

hold on
yline(del_d_sol_x-0.004,'--','Color',[0.000 0.447 0.741],'LineWidth',1.4)
yline(del_d_sol_y+0.004,'--','Color',[0.850 0.325 0.098],'LineWidth',1.4)
legend("$\Delta\,d_x$","$\Delta\,d_y$","","")
xlabel("Time [s]")


fontsize(13,'points')

exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\adaptive_sim_success.pdf','ContentType','vector');        % axes only

%% Failure when using ekf

f = gen_single_fig();
plot(out.tout,omega_p_plt(1:3,:),'LineWidth',1.5)
grid on
xlabel("Time [s]")
ylabel("Projected Body Rates [rad/s]")
legend("$\omega_{p,x}$","$\omega_{p,x}$","$\omega_{p,z}$")
fontsize(13,'points')
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\adaptive_sim_failure.pdf','ContentType','vector');        % axes only