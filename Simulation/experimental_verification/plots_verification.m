%% Torque
clc; clear

f = gen_single_fig(); grid on; hold on;

load("manual_balance_torque.mat")
% plot(t_al,torque_norm,'LineWidth',1.5);

clc; clear
load("hybrid_torque.mat")
plot(0.1.*linspace(0,590,591),torque_norm(550:1140))
% plot(t_al,torque_norm,'LineStyle',1.5);

fontsize(13,'points')

%%


%% Kinetic Energy

clc; clear
f = gen_single_fig(); grid on; hold on;
load("manual_balance_KE.mat")
plot(t_plt,KE,'LineWidth',1.5);
clc; clear
load("passive_KE.mat")
plot(t_plt,KE,'LineWidth',1.5);
fontsize(13,'points')