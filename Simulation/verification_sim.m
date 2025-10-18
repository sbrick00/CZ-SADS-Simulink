%%
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = 1e-1.*[3*10^-3, 2*10^-3 ,-3*10^-2]';
S.t_sim = 100;
S.EA_0 = deg2rad([6 ,6, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';
out = simFromStruct('main_sim',S);

n = length(out.tout);
KE = zeros(n,1);
torque = zeros(n,1);
omega_b =  squeeze(out.omega_b.signals.values);
tau_g_0 = squeeze(out.tau_g_0.signals.values);
for i = 1:n
  omega = omega_b(i,:)';
  KE(i) = omega'*S.J_0*omega;
  torque(i) = norm(tau_g_0(:,i));

end

disp("Mean KE: " + mean(KE))
disp("Mean Torque: " + mean(torque))