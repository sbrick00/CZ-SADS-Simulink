%%
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
% S.r_0 = [4*10^-3, 2*10^-3 ,-3*10^-2];
S.r_0 = [-7.2931e-04 4e-04 -3*10^-2];
S.t_sim = 64;
S.omega_0 = [-0.01 0.00 0.06]';
S.EA_0 = deg2rad([-3 ,4, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';

gainScale = 0.5;
S.K_omega = gainScale*1;
S.K_q = gainScale*1;
S.K_I = gainScale*6;


out = simFromStruct('main_sim',S);
r_b = squeeze(out.r_b.signals.values);
final_r_b = r_b(:,end);
disp("Final r_b: ")
disp(final_r_b(:,end))

EA_plt = deg2rad(squeeze(out.EA_degs.signals.values));
omega_b_plt = squeeze(out.omega_b.signals.values);
del_r_plt = squeeze(out.del_r.signals.values);
t_plt = out.tout;

del_d_plt = (del_r_plt)*(S.m_s/S.mmu);
%%
f = gen_tiled_layout(3);
t = tiledlayout(f,3,1,'Padding','compact','TileSpacing','compact');
ax1 = nexttile;

plot(out.tout,EA_plt(:,1:2),'LineWidth',1.5)
grid on
ylabel("Euler Angles [rad]")
legend("$\phi$","$\theta$")

ax2 = nexttile;
plot(out.tout,omega_b_plt(:,1:2),'LineWidth',1.5)
grid on
ylabel("Body Rates [rad/s]")
legend("$\omega_x$","$\omega_y$")

ax3 = nexttile;
plot(out.tout,del_d_plt(:,1:2),'LineWidth',1.5)
grid on 
ylabel("Sliding Mass Positions [m]")
legend("$\Delta\,d_x$","$\Delta\,d_y$")
xlabel("Time [s]")


fontsize(13,'points')

f = gen_single_fig();
plot(out.tout,squeeze(out.proj_int_action.signals.values),'LineWidth',1.5)

grid on
fontsize(13,'points')
xlabel('Time [s]')
ylabel('Integral Error [N$\cdot$m]')
