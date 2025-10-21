%% Torque
clc; clear

f = gen_single_fig(); grid on; hold on;

load("manual_balance_torque.mat")
plot(t_al,torque_norm,'LineWidth',1.5);

disp("Mean torque 1: " + mean(torque_norm))

load("passive_torque.mat")
plot(t_al,torque_norm,'LineWidth',1.5)
xlabel("Time [s]")
ylabel("Torque [N$\cdot$m]")
legend("Manual Balancing","PID Balancing")
fontsize(13,'points')

disp("Mean torque 2: " + mean(torque_norm))
%exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\hardware_verification_torque.pdf','ContentType','vector');        % axes only


%%


%% Kinetic Energy

clc; clear
f = gen_single_fig(); grid on; hold on;
load("manual_balance_KE.mat")
plot(t_plt,KE,'LineWidth',1.5);


disp("Max Delta KE 1: " + abs(max(KE)-min(KE)))


load("passive_KE.mat")
plot(t_plt,KE,'LineWidth',1.5);

disp("Max Delta KE 2: " + abs(max(KE)-min(KE)))
fontsize(13,'points')