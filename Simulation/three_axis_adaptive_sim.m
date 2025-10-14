%%
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = 1e-3.*[9*10^-3,14*10^-3 ,-3*10^-2]';
S.t_sim = 1000;
S.Gamma = 0.001*diag([0.005 0.005 0.015] );
S.EA_0 = deg2rad([-5 ,-14, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';
out = simFromStruct('main_sim',S);

%%
f = gen_tiled_layout(3);
t = tiledlayout(f,3,1,'Padding','compact','TileSpacing','compact');



omega_b_plt = squeeze(out.omega_b.signals.values);
wheel_speeds_plt = squeeze(out.wheel_speeds_RPM.signals.values);
EA_plt = squeeze(out.EA_degs.signals.values);
r_b_plt = squeeze(out.r_b.signals.values);


t_start = 17;
start_idx = find(out.tout > t_start, 1, 'first');
t_0 = out.tout(start_idx);
start_idx_wheels = find(out.wheel_speeds_RPM.time > t_start, 1, 'first');
t_0_wheels = out.wheel_speeds_RPM.time(start_idx_wheels);
%
del_d_plt = (r_b_plt-r_b_plt(:,start_idx))*(S.m_s/S.mmu);
d_sol = -r_b_plt(:,start_idx)*(S.m_s/S.mmu);
ax1 = nexttile;
plot(out.tout(start_idx:end)-t_0,deg2rad(EA_plt(:,start_idx:end)),'LineWidth',1.5)
grid on
ylabel("Euler Angles [rad]")
legend("$\phi$","$\theta$","$\psi$")

ax2 = nexttile; 
plot(out.tout(start_idx:end)-t_0,omega_b_plt(:,start_idx:end),'LineWidth',1.5)
grid on
ylabel("Body Rates [rad/s]")
legend("$\omega_x$","$\omega_y$","$\omega_z$")

ax3 = nexttile;
plot(out.wheel_speeds_RPM.time(start_idx_wheels:end)-t_0_wheels, ...
     wheel_speeds_plt(:,start_idx_wheels:end), ...
     'LineWidth',1.5)

grid on
ylabel("Reaction Wheel Speeds [RPM]")
xlabel("Time [s]")
legend("Wh. 1","Wh. 2","Wh. 3","Wh. 4")


%
f = gen_side_by_side_fig;
% plot(0.7.*(out.tout(start_idx:end)-t_0),del_d_plt(:,start_idx:end),'LineWidth',1.5)
plot(out.tout(start_idx:end)-t_0,r_b_plt(:,start_idx:end),'LineWidth',1.5)
grid on
hold on
% yline(d_sol(1),'--','LineWidth',1.1,'Color',[0.000 0.447 0.741])
% yline(d_sol(2),'--','LineWidth',1.1,'Color',[0.850 0.325 0.098])
% yline(d_sol(3),'--','LineWidth',1.1,'Color',[0.13, 0.54, 0.13])

fontsize(13,'points')
legend("$\Delta\,d_x$","$\Delta\,d_y$","$\Delta\,d_z$","Location","northwest")
xlabel("Time [s]")
ylabel("Sliding Mass Positions [m]")



ax = gca;
ax.TickLabelInterpreter = 'latex';
% ax.YAxis.Exponent = -2; 
pad = 0.015;   % inches-ish; small but safe for labels/ticks
ax.LooseInset = max(ax.TightInset, pad*[1 1 1 1]);