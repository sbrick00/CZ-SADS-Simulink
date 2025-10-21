%%
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = 1e-3.*[0.1478 0.1345 -0.3815]';
S.t_sim = 100;
S.omega_0 = [0 0 0]';
S.EA_0 = deg2rad([6 ,6, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';

% S.J_0 = diag([0.6 0.65 0.65]);
% S.m_s = 23;
out = simFromStruct('main_sim',S);

n = length(out.tout);
KE = zeros(n,1);
torque = zeros(n,1);
omega_b =  squeeze(out.omega_b.signals.values)';
tau_g_0 = squeeze(out.tau_g_0.signals.values);
for i = 1:n
  omega = omega_b(:,i);
  KE(i) = omega'*S.J_0*omega;
  torque(i) = norm(tau_g_0(:,i));

end

disp("Max Delta KE: " + abs(max(KE)-min(KE)))
disp("Mean Torque: " + mean(torque))

figure
plot(out.tout,squeeze(out.EA_degs.signals.values))