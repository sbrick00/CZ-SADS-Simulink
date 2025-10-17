clc; clear
load("hybrid_verification4.mat")
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


t_start = 11;
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

%%
vel_x = bodyRates_plt(:,1);

% Design a FIR filter to differentiate --------------------------------
figure
pwelch(vel_x,[],[],[],sampleRate)
Fpass = 0.05;
Fstop = 0.3;
Fs = sampleRate;

d = designfilt('differentiatorfir', ...
    'PassbandFrequency', Fpass, ...
    'StopbandFrequency', Fstop, ...
    'FilterOrder',100, ... 
    'SampleRate', Fs);

accel_x = filter(d,vel_x)/sampleTime;
delay = mean(grpdelay(d));

t_d = t(1:end-delay);

accel_x_d = accel_x;
accel_x_d(1:delay) = [];

t_d(1:delay) = [];
accel_x_d(1:delay) = [];

plot(t_plt,accel_x)
xlabel('Time (s)')
ylabel('Angular Acceleration [rad/s]')
yline(0)
xlim([t_d(1) t_d(end)])
grid


%%
figure
plot(t,eulerAngles,'LineWidth',1.5);
f = gen_single_fig();
% plot(t_plt,bodyRates_plt,'LineWidth',1.5);
plot(t_plt,KE,'LineWidth',1.5)
grid on
fontsize(13,'points')
xlabel("Time [s]")
ylabel("Kinetic Energy [J]")

save("hybrid_4_KE.mat","t_plt","KE")

%%
f = gen_single_fig;
load("manual_balance_KE.mat");
plot(t_plt,KE,'LineWidth',1.5)

hold on

load("hybrid_4_KE.mat");
plot(t_plt,KE,'LineWidth',1.5)

grid on
fontsize(13,'points')
xlabel("Time [s]")
ylabel("Kinetic Energy [J]")
legend("Manual Balancing","PID Balancing")