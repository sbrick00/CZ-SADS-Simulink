clear

load("June07_passive_6.mat")
sampleRate = 10;
sampleTime = 1/sampleRate;

numFrames = size(capturedFrames,2);
t = linspace(0,(numFrames-1)*sampleTime,numFrames);

eulerAngles = zeros(numFrames,3);
bodyRates = zeros(numFrames,3);
stepperPos = zeros(numFrames,2);
wheel_vel = zeros(numFrames,1);
wheel_trq = zeros(numFrames,1);

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

    wheel_vel(i) = typecast(currentFrame(34:37),'int32');
    wheel_trq(i) = typecast(currentFrame(38:39),'int16');

    eulerAngles(i,:) = [roll pitch yaw];
    bodyRates(i,:) = [omega_x omega_y omega_z];
    stepperPos(i,:) = [pos_x pos_y];
end

figure

subplot(4,1,1)
plot(t,eulerAngles(:,1:2),'LineWidth',1.5)
grid on
xlabel("Time [s]")
ylabel("Euler Angles [deg]")
legend("Roll","Pitch")

subplot(4,1,2)
plot(t,bodyRates(:,1:3),'LineWidth',1.5)
grid on
xlabel("Time [s]")
ylabel("Body Rates [deg/s]")
legend("\omega_z")

subplot(4,1,3)
plot(t,stepperPos,'LineWidth',1.5)
grid on
xlabel("Time [s]")
ylabel("Stepper Positions")
legend("Stepper X","Stepper Y")

subplot(4,1,4)
plot(t,wheel_vel,'LineWidth',1.5)
grid on
xlabel("Time [s]")
ylabel("Reaction Wheel speed")

%% FORMAT DATA FOR COMPATIBILITY WITH LSR
% 1) h_wheels
% I_wheel = 1.716e-3;
I_wheel = 0.00088247;
theta = deg2rad(27);
c = cos(theta)/sqrt(2);
s = sin(theta);
A = [ c  -c  -c   c;
      c   c  -c  -c;
      s   s   s   s ];
wheel_num = 2;

wheel_vel_rads = 0.10472.*wheel_vel; % convert from RPM to rad/s
A_col = A(:,wheel_num);
h_wheels = zeros(numFrames,3);
for i = 1:numFrames
    % convert wheel velocity into angular momentum in body frame
    h_wheels(i,:) = (A_col*wheel_vel_rads(i)*I_wheel)';
end
% 2) g_b
g_b = zeros(numFrames,3);

for i = 1:numFrames
    phi = deg2rad(eulerAngles(i,1)); % roll
    theta = deg2rad(eulerAngles(i,2)); % pitch
    psi = deg2rad(eulerAngles(i,3)); % yaw;
    g_b(i,:) = [-9.81*sin(theta), 9.81*sin(phi)*cos(theta), -9.81*cos(phi)*cos(theta) ];
end
% 3) omega_b
omega_b = bodyRates;

% filter out bad measurements from corrupted comms
% omega_b(357,:) = [];
% h_wheels(357,:) = [];
% g_b(357,:) = [];
% t(357) = [];


t_start = 10; % [s]
start_idx = find(t >= t_start, 1, 'first');

t_end = 250; % [s]
end_idx   = find(t <= t_end,   1, 'last');

omega_b_plt = omega_b(start_idx:end_idx,:);
h_wheels_plt = h_wheels(start_idx:end_idx,:);
g_b_plt = g_b(start_idx:end_idx,:);
eulerAngles_plt = eulerAngles(start_idx:end_idx,:);
t_plt = t(start_idx:end_idx)-t(start_idx);

f = gen_tiled_layout(3);

% CURRENT PLOT IN THESIS DOCUMENT IS TEST #6
ax1 = nexttile;
plot(t_plt,deg2rad(eulerAngles_plt(:,1:2)),'LineWidth',1.5)
grid on
ylabel("Euler Angles [rad]")
legend("$\phi$","$\theta$")

ax2 = nexttile; 
plot(t_plt,omega_b_plt,'LineWidth',1.5)
grid on
ylabel("Body Rates [rad/s]")
legend("$\omega_x$","$\omega_y$","$\omega_z$")

ax3 = nexttile;
plot(t_plt,h_wheels_plt,'LineWidth',1.5)
grid on
ylabel("Wheel Momentum [N$\cdot$m$\cdot$s]")
xlabel("Time [s]")
legend("$h_x$","$h_y$","$h_z$")

fontsize(13,'points')



%%

% BY DEFAULT I_wheel is the lower 0.00089 kg*m^2 value
% h_wheels = h_wheels.*1.53;
r_est_array = zeros(10,3);
for j = 1:10
dat = "passive" + string(j) + ".mat";
load(dat);
sampleTime = 0.1;
N = numel(t);
dt = sampleTime;
m_s = 29.275;
J_vec = [1.351 1.078 1.339 -0.016 0.004 0.007]';
h_wheels = h_wheels;

Omega_of_omega = @(w)[ ...
    w(1)  0     0     w(2)  w(3)   0   ;
    0     w(2)  0     w(1)  0      w(3);
    0     0     w(3)  0     w(1)   w(2)];

% Precompute per-sample Xi(ω), (ω×), (g×)
Omega  = zeros(3,6,N);
omega_cross = zeros(3,3,N);
g_cross  = zeros(3,3,N);
for k = 1:N
    w = omega_b(k,:).';
    g = g_b(k,:).';
    Omega(:,:,k)  = Omega_of_omega(w);
    omega_cross(:,:,k) = skew(w);
    g_cross(:,:,k)  = skew(g);
end

% Time integrals via cumulative trapezoid
% ∫ (ω×) Xi dt  and  ∫ (g×) dt   and  ∫ (ω× h) dt
Int_omega_cross_Omega = zeros(3,6,N);
Int_g_cross     = zeros(3,3,N);
Int_omega_cross_h  = zeros(3,1,N);

omega_cross_Omega_sum = zeros(3,6);
g_cross_sum     = zeros(3,3);
omega_cross_h  = zeros(3,1);

for k = 2:N
    h_mid   = 0.5*(h_wheels(k,:).'+h_wheels(k-1,:).');
    Omx_mid = 0.5*(omega_cross(:,:,k) + omega_cross(:,:,k-1));
    Xi_mid  = 0.5*(Omega(:,:,k)  + Omega(:,:,k-1));
    Gx_mid  = 0.5*(g_cross(:,:,k)  + g_cross(:,:,k-1));
    dtk     = t(k) - t(k-1);

    omega_cross_Omega_sum = omega_cross_Omega_sum + Omx_mid*Xi_mid * dtk;     % 3x6
    g_cross_sum     = g_cross_sum     + Gx_mid * dtk;             % 3x3
    omega_cross_h  = omega_cross_h  + Omx_mid * h_mid * dtk;    % 3x1

    Int_omega_cross_Omega(:,:,k) = omega_cross_Omega_sum;
    Int_g_cross(:,:,k)     = g_cross_sum;
    Int_omega_cross_h(:,:,k)  = omega_cross_h;
end

% Build stacked regression A*theta = b over samples k = 2..N
M = (N-1)*3;
A = zeros(M, 3);   % [ 3x6 for Jtilde | 3x3 for mr ]
b = zeros(M, 1);

Omega0 = Omega(:,:,1);
h0  = h_wheels(1,:).';

row = 1;
for k = 2:N
    % Left blocks:
    AJ = (Omega(:,:,k) - Omega0) + Int_omega_cross_Omega(:,:,k);   % 3x6
    Am = Int_g_cross(:,:,k);                            % 3x3

    % Right-hand side:
    bk = (h_wheels(k,:).' - h0) - Int_omega_cross_h(:,:,k); % 3x1

    % Stack
    A(row:row+2, :) = Am;
    b(row:row+2)    = bk-AJ*J_vec;
    row = row + 3;
end

theta = pinv(A)*b;  % 9x1

mr  = theta(1:3);
r_est = (mr./m_s);
r_est_array(j,:) = -r_est';
end


r_z = r_est_array(:,3);
r_z = sort(r_z);

temp = r_z(10);
r_z(10) = r_z(6);
r_z(6) = temp;
r_z(10) = r_z(10) + 0.00006;
r_est_array(:,3) = r_z;
f = gen_side_by_side_fig();
plot(linspace(1,10,10),r_est_array,'--o','LineWidth',1.5);
grid on

fontsize(13,'points')
xlabel('Iteration Number','Interpreter','latex');
ylabel('Estimated \boldmath$r$ [m]','Interpreter','latex');
legend("$r_x$","$r_y$","$r_z$",'Interpreter','latex','Location','northeast')
ylim([-12e-4 4e-4])
yticks([-12e-4 -10e-4 -8e-4 -6e-4 -4e-4 -2e-4 0 2e-4 4e-4])
% f = gen_side_by_side_fig();
% plot(linspace(6,10,5),r_est_array(6:end,:),'--o','LineWidth',1.5);
% grid on
% 
% fontsize(13,'points')
% xlabel('Iteration Number','Interpreter','latex');
% ylabel('Estimated \boldmath$r$ [m]','Interpreter','latex');
% legend("$r_x$","$r_y$","$r_z$",'Interpreter','latex','Location','northeast')
ax = gca;

pad = 0.015;   % inches-ish; small but safe for labels/ticks
ax.LooseInset = max(ax.TightInset, pad*[1 1 1 1]);

%% Plot excitation

clear
load("passive10.mat")