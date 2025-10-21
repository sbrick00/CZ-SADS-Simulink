%%
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = [3*10^-3, 2*10^-3 ,-3*10^-2]';
S.omega_0 = [0 0 0]';
S.Kappa_LSR = .005*diag([1 1 1]);
S.t_sim = 400;
out = simFromStruct('main_sim',S);
%% Run Passive Iterations
num_tests = 20;
r_true = zeros(num_tests,3);
r_est_array = zeros(num_tests,3);
J_0 = S.J_0;
m_s = S.m_s;
J_vec = [0.93*J_0(1,1),1.07*J_0(2,2),0.87*J_0(3,3),J_0(1,2),J_0(1,3),J_0(2,3)]';

% BEGIN TEST LOOP
for j = 1:num_tests % ------------------------------------------------------
disp("Starting test " + j)
out = simFromStruct('main_sim',S);

t = out.tout;
use_EKF = 1; % use EKF measurements or truth sim data
if use_EKF == 1
    g_b = squeeze(out.g_b_EKF.signals.values)';
    h_wheels = squeeze(out.h_wheels.signals.values)';
    omega_b = squeeze(out.omega_EKF.signals.values)';
    t = out.omega_EKF.time;
    N = numel(t);
    dt = [0; diff(t)];
else 
    LSR_sample_time = 0.1;
    omega_b = out.omega_b.signals.values;
    g_b = squeeze(out.g_b.signals.values)';
    tau_c = out.tau_truth.signals.values;
    h_wheels = squeeze(out.h_wheels.signals.values)';
    % rewrite signals to only sample every sample_time seconds
    t0 = t(1);
    tf = t(end);
    t_grid = (t0:LSR_sample_time:tf).';
    
    % Use 'previous' (ZOH) or 'linear' depending on what you want
    method = 'previous';  % or 'linear'
    
    omega_b  = interp1(out.omega_b.time, omega_b,  t_grid, method, 'extrap');
    g_b      = interp1(out.g_b.time, g_b,      t_grid, method, 'extrap');
    tau_c    = interp1(out.tau_truth.time, tau_c,    t_grid, method, 'extrap');
    h_wheels = interp1(out.h_wheels.time, h_wheels, t_grid, method, 'extrap');
    t        = t_grid;
    N = numel(t);
    dt = [0; diff(t)];
end

h_wheels = -h_wheels;

Omega_of_omega = @(w)[ ...
    w(1)  0     0     w(2)  w(3)   0   ;
    0     w(2)  0     w(1)  0      w(3);
    0     0     w(3)  0     w(1)   w(2)];

% Precompute per-sample
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

% Time integrals with trapezoid method
Int_omega_cross_Omega = zeros(3,6,N);
Int_g_cross     = zeros(3,3,N);
Int_omega_cross_h  = zeros(3,1,N);

omega_cross_Omega_sum = zeros(3,6);
g_cross_sum     = zeros(3,3);
omega_cross_h  = zeros(3,1);

for k = 2:N
    h_mid   = 0.5*(h_wheels(k,:).'+h_wheels(k-1,:).');
    omega_cross_mid = 0.5*(omega_cross(:,:,k) + omega_cross(:,:,k-1));
    Xi_mid  = 0.5*(Omega(:,:,k)  + Omega(:,:,k-1));
    g_cross_mid  = 0.5*(g_cross(:,:,k)  + g_cross(:,:,k-1));
    dt     = t(k) - t(k-1);

    omega_cross_Omega_sum = omega_cross_Omega_sum + omega_cross_mid*Xi_mid * dt;     % 3x6
    g_cross_sum     = g_cross_sum     + g_cross_mid * dt;             % 3x3
    omega_cross_h  = omega_cross_h  + omega_cross_mid * h_mid * dt;    % 3x1

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


r_est_array(j,:) = (r_est)';
S.r_0 = S.r_0 - r_est;

r_true(j,:) = S.r_0';
end % END TEST LOOP -------------------------------------------------
%%
sig_x = sqrt(var(r_est_array(5:end,3)));
%% Plot Results


start_test = 5;
end_test = 20;
test_plotted = end_test-start_test + 1;
lin = linspace(start_test,end_test,test_plotted);

% f = gen_side_by_side_fig();
% plot(linspace(start_test,end_test,test_plotted),-r_est_array(start_test:end_test,:),'--o','LineWidth',1.5)

f = gen_single_fig(); hold on;
plot(lin,-r_est_array(start_test:end_test,3),'-o','LineWidth',1.5)

plot(lin,-r_est_array(start_test:end_test,3)+3*sig_x,'--o','LineWidth',1.5,'Color',[0.000 0.447 0.741])
plot(lin,-r_est_array(start_test:end_test,3)-3*sig_x,'--o','LineWidth',1.5,'Color',[0.000 0.447 0.741])

plot(lin,r_true(start_test:end_test,1) ...
        ,'-o','LineWidth',1.5,'Color',[0.200 0.200 0.200])

grid on

fontsize(13,'points')
xlabel('Iteration Number','Interpreter','latex');
ylabel('$r_x$ [m]','Interpreter','latex');
legend("$r_x$ Est.","$r_x$ Est. $\pm3\sigma$","","$r_x$ Truth",'Interpreter','latex','Location','northeast')
ax = gca;
ax.TickLabelInterpreter = 'latex';

pad = 0.015;   % inches-ish; small but safe for labels/ticks
ax.LooseInset = max(ax.TightInset, pad*[1 1 1 1]);

exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\LSR_sim_confidence_1_wheel.pdf','ContentType','vector');        % axes only

%% Test excitation
S = setup();            

% optionally overwrite default values
S.r_0 = 1e-3.*[6*10^-3, 7*10^-3 ,-3*10^-2]';
S.omega_0 = [0 0 0]';
S.Kappa_LSR = .005*diag([1 1 1]);
S.t_sim = 250;
out = simFromStruct('main_sim',S);
%


%pad = 0.015;   % inches-ish; small but safe for labels/ticks
%ax.LooseInset = max(ax.TightInset, pad*[1 1 1 1]);

f = gen_tiled_layout(3);
t = tiledlayout(f,3,1,'Padding','compact','TileSpacing','compact');

omega_b_plt = squeeze(out.omega_b.signals.values);
wheel_speeds_plt = squeeze(out.wheel_speeds_RPM.signals.values);
EA_plt = squeeze(out.EA_degs.signals.values(:,1:3));

ax1 = nexttile;
plot(out.tout,deg2rad(EA_plt),'LineWidth',1.5)
grid on
ylabel("Euler Angles [rad]")
legend("$\phi$","$\theta$","$\psi$")

ax2 = nexttile; 
plot(out.tout,omega_b_plt,'LineWidth',1.5)
grid on
ylabel("Body Rates [rad/s]")
legend("$\omega_x$","$\omega_y$","$\omega_z$")

ax3 = nexttile;
plot(out.wheel_speeds_RPM.time,wheel_speeds_plt,'LineWidth',1.5)
grid on
ylabel("Reaction Wheel Speeds [RPM]")
xlabel("Time [s]")
legend("Wh. 1","Wh. 2","Wh. 3","Wh. 4")

fontsize(13,'points')