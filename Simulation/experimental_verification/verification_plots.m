clc; clear
load("passive_verification3.mat")
% load("manual_balance_tumbling.mat")
%{
passive verification 1-3 contain tumbling data after all 10 passive runs,
all with varying initial conditions
    1. roll = 6.5, pitch = -6.5
    2. probably dont use
    3. roll+pitch = 6

should be compared to manaual balance tumbling

%}

J = [1.298  -0.014 0.004;
      -0.014  1.025  0.008;
       0.004  0.008 1.243];

sampleRate = 10;
sampleTime = 1/sampleRate;

numFrames = size(capturedFrames,2);
t = linspace(0,(numFrames-1)*sampleTime,numFrames);

eulerAngles = zeros(numFrames,3);
bodyRates = zeros(numFrames,3);
stepperPos = zeros(numFrames,2);


%
% unpacking data...
for i = 1:numFrames % <---  REMEMBER TO INCLUDE/NOT INCLUDE RW WHEEL DATA HERE!!
    currentFrame = capturedFrames(:,i)';
    
    roll = typecast(currentFrame(2:5),'single');
    pitch = typecast(currentFrame(6:9),'single');
    yaw = typecast(currentFrame(10:13),'single');
    
    omega_x = typecast(currentFrame(14:17),'single');
    omega_y = typecast(currentFrame(18:21),'single');
    omega_z = typecast(currentFrame(22:25),'single');
    
    pos_x = typecast(currentFrame(26:29),'int32');
    pos_y = typecast(currentFrame(30:33),'int32');

    eulerAngles(i,:) = [roll pitch yaw];
    bodyRates(i,:) = [omega_x omega_y omega_z];
    stepperPos(i,:) = [pos_x pos_y];
end


%% FIND GOOD STARTING POINT AND SAVE KE
figure
plot(t,eulerAngles)

t_start = 14;
start_idx = find(t > t_start, 1, 'first');
t_0 = t(start_idx);

t_plt = t(start_idx:end)-t_0;
eulerAngles_plt = deg2rad(eulerAngles(start_idx:end,1:3));
bodyRates_plt = bodyRates(start_idx:end,1:3);


n = length(t_plt);
KE = zeros(n,1);
for i = 1:n
    omega = bodyRates_plt(i,:)';
    KE(i) = (1/2)*omega'*J*omega;
end

save("passive_KE.mat","KE","t_plt")

%% SAVE TORQUE
vel_x = bodyRates_plt(:,1);
vel_y = bodyRates_plt(:,2);
vel_z = bodyRates_plt(:,3);

% Design a FIR filter to differentiate --------------------------------
% figure
% pwelch(vel_x,[],[],[],sampleRate) 

Fpass = 0.05;
Fstop = 0.3;
Fs = sampleRate;

d = designfilt('differentiatorfir', ...
    'PassbandFrequency', Fpass, ...
    'StopbandFrequency', Fstop, ...
    'FilterOrder',100, ... 
    'SampleRate', Fs);

accel_x = filter(d,vel_x)/sampleTime;
accel_y = filter(d,vel_y)/sampleTime;
accel_z = filter(d,vel_z)/sampleTime;

gd = round(mean(grpdelay(d)));     % constant group delay for linear-phase FIR
t_al = t_plt(1:end-gd);
vel_x_al   = vel_x(1:end-gd);
vel_y_al   = vel_y(1:end-gd);
vel_z_al   = vel_z(1:end-gd);
accel_x_al = accel_x(1+gd:end);
accel_y_al = accel_y(1+gd:end);
accel_z_al = accel_z(1+gd:end);

figure; hold on; grid on
yline(0)
plot(t_al, vel_x_al);
plot(t_al, accel_x_al);
legend("velocity","acceleration"); 
xlabel('Time (s)')
title("pick good t_start for saving torque")

torque = zeros(length(t_al),3);
torque_norm = zeros(length(t_al),1);
for i = 1:length(t_al)
    omega_dot = [accel_x_al(i), accel_y_al(i), accel_z_al(i)]';
    omega = [vel_x_al(i), vel_y_al(i), vel_z_al(i)]';
    torque(i,:) = J*omega_dot + cross(omega,J*omega);
    torque_norm(i) = norm(torque(i,:));
end

t_start = 5.4;
start_idx = find(t_al > t_start, 1, 'first');
t_0 = t_al(start_idx);

t_al = t_al(start_idx:end)-t_0;
torque = torque(start_idx:end,:);
torque_norm = torque(start_idx:end);
figure; hold on; grid on
yline(0)
plot(t_al, torque);
t_plt = t(start_idx:end)-t_0;
title("Torque")
save("passive_torque.mat","t_al","torque_norm")


%% PLOT RESUTLS
f = gen_single_fig;
load("manual_balance_KE.mat");
plot(t_plt,KE,'LineWidth',1.5)
disp("Mean KE for manual: " + mean(KE))
hold on

load("hybrid_4_KE.mat");
plot(t_plt,KE,'LineWidth',1.5)

grid on
fontsize(13,'points')
xlabel("Time [s]")
ylabel("Kinetic Energy [J]")
legend("Manual Balancing","PID Balancing")
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\hardware_verification_KE.pdf','ContentType','vector');        % axes only
