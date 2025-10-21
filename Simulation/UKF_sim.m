%% 
clc; clear;
addpath("sim_helpers\")
S = setup();            

% optionally overwrite default values
S.r_0 = [1*-6e-06 1*9e-06 -2e-9];
S.t_sim = 100;
S.omega_0 = [-0.01 0.04 0.15]';
S.EA_0 = deg2rad([-18 ,21, 0]');
S.q_0 = eul2quat(S.EA_0',"XYZ")';
omega_0_init = [0 0 0]';
q_0_init = [1 0 0 0]';
r_z_init = 3*S.r_0(3);
S.x_0_ukf = [omega_0_init; S.q_0; r_z_init];

Q_omega = 5e-5;
Q_q = 4e-6;
Q_r_z = 5e-13;

S.Q_ukf  = diag([Q_omega Q_omega Q_omega Q_q Q_q Q_q Q_q Q_r_z]);

out = simFromStruct('main_sim',S);

f = gen_single_fig();

start_idx = 3;
t = out.r_z_est.time(start_idx:end);
r_z_UKF = squeeze(out.r_z_est.signals.values(start_idx:end));
sigma_UKF = squeeze(out.sigma_r_z.signals.values(start_idx:end));

plot(t,r_z_UKF,'LineWidth',1.5)
hold on
plot(t,r_z_UKF+sigma_UKF,'--','Color',[0.000 0.447 0.741])
plot(t,r_z_UKF-sigma_UKF,'--','Color',[0.000 0.447 0.741])
grid on
fontsize(13,'points');
hold on
yline(S.r_0(3),'--','LineWidth',1.5)
xlabel("Time [s]")
ylabel("Estimated $r_z$ [m]")

legend("Estimated Value","$1\sigma$","","True Value")

disp("Predicted: " + r_z_UKF(end));
disp("True: " + S.r_0(3))
disp("Error: " + (r_z_UKF(end) - S.r_0(3)))
disp("1 sigma: " + sigma_UKF(end))
disp("CV: " + (100*sigma_UKF(end)/abs(r_z_UKF(end))))
disp("Corresponding steps: " + r_z_UKF(end)*(S.m_s/S.mmu)*1e3*3200)

%%
r_z_array_exp = [-2.0928e-03, -6.3928e-04, -3.0928e-04,-1.0928e-04];
sigma_array_exp = [1.0928e-04, 0.6e-04, -1.0928e-04,-1.5928e-04];

r_z_array = [-2.0928e-03, -9.4876e-05, -1.4876e-05,-9.4876e-65];
sigma_array = [6.0928e-05, 4.4744e-05, 2.4744e-05, 2.4744e-05];

n = length(r_z_array);
f = gen_side_by_side_fig();
% plot(linspace(1,n,n),r_z_array,'--o','Color',[0.000 0.447 0.741],'LineWidth',1.5);
yline(0)
hold on
errorbar(linspace(1,n,n),r_z_array_exp,sigma_array_exp,'--o','LineWidth',1.5)
errorbar(linspace(1,n,n),r_z_array,sigma_array,'--o','LineWidth',1.5)
legend("","Experimental","Simulation","Location","southeast")
xlabel("Iteration Number")
ylabel("Estimated $r_z\pm1\sigma$ [m]")
grid on
fontsize(13,'points')
xlim([0 n+1])
xticks(linspace(0,5,6))
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\UKF_comparison.pdf','ContentType','vector');        % axes only

%{
function [x_kn1_i,Wm,Wc] = calc_sigma_points(x_kn1, P_kn1)

n = size(x_kn1,1);

alpha   = 1e-3;         % small, positive
kappa   = 0;            % usually 0
beta    = 2;            % optimal for Gaussian


lambda  = alpha^2*(n+kappa) - n;

P_kn1 = 0.5*(P_kn1 + P_kn1.');         % symmetrize


x_kn1_i = zeros(n,2*n + 1);
x_kn1_i(:,1) = x_kn1; % central point

for i = 1:n
    x_kn1_i(:,i+1)   = x_kn1 + root(:,i);
    x_kn1_i(:,i+1+n) = x_kn1 - root(:,i);
end
      

Wm      = 0.5 /(n+lambda) * ones(2*n+1,1);
Wc      = Wm;
Wm(1)   = lambda/(n+lambda);
Wc(1)   = Wm(1) + (1-alpha^2+beta);

function x_k_i = f(x_kn1_i,T,J,m_s)
    n = size(x_kn1_i,1);
    x_k_i = zeros(n,2*n+1);
    for i = 1:2*n+1
        omega_kn1 = x_kn1_i(1:3,i);
        eta_kn1 = x_kn1_i(4,i);
        eps_kn1 = x_kn1_i(5:7,i);

        q = [eta_kn1; eps_kn1;];
        % Gravity vector in body-frame
        g_b = [2*q(1)^2-1+2*q(2)^2 2*q(2)*q(3)-2*q(1)*q(4) 2*q(2)*q(4)+2*q(1)*q(3);
                2*q(2)*q(3)+2*q(1)*q(4) 2*q(1)^2-1+2*q(3)^2 2*q(3)*q(4)-2*q(1)*q(2);
                2*q(2)*q(4)-2*q(1)*q(3) 2*q(3)*q(4)+2*q(1)*q(2) 2*q(1)^2-1+2*q(4)^2]' * [0; 0; -9.81];

   
        r_z_kn1 = x_kn1_i(8,i);
        
        r_kn1 = [0 0 r_z_kn1]';
        
        x_k_i(:,i) = x_kn1_i(:,i) + T*[J\(-skew(omega_kn1)*J*omega_kn1 + m_s*skew(r_kn1)*g_b); 
                                     -.5*eps_kn1'*omega_kn1;
                                      .5*(eta_kn1*eye(3) + skew(eps_kn1))*omega_kn1; 
                                       0];
        
        % normalize the quaternion portion of the state
        x_k_i(4:7,i) = x_k_i(4:7,i)./norm(x_k_i(4:7,i)); 
    end
end

function [y_hat_k,y_k_i]   = predict_measurements(Wm,x_k_i,H)
    y_k_i = H * x_k_i;
    
    y_hat_k = y_k_i * Wm;
end

function [x_k_minus,P_k_minus] = predict_state_and_covariance(x_k_i,Wm,Wc,Q)

    x_k_minus = x_k_i * Wm;  % weighted mean  (matrix‑vector product)
    x_k_minus(4:7) = x_k_minus(4:7)./norm(x_k_minus(4:7));
    
    P_k_minus = (x_k_i - x_k_minus)*diag(Wc) * (x_k_i - x_k_minus)'+ Q;  
end

function [P_y,P_xy] = calc_Py_and_Px(y_hat_k,y_k_i,x_k_i,x_k_minus,R,Wc)
    
    diff_y = y_k_i - y_hat_k;
    diff_x = x_k_i - x_k_minus;
    
    P_y = diff_y * diag(Wc) * diff_y' + R;
    P_xy = diff_x * diag(Wc) * diff_y';
end

function [x_k_plus,P_k_plus,sigma_r_z] = correct(y_k, y_hat_k, P_y,P_xy,x_k_minus,P_k_minus)

K = P_xy/P_y;

    x_k_plus = x_k_minus + K*(y_k-y_hat_k);
    x_k_plus(4:7) = x_k_plus(4:7)./norm(x_k_plus(4:7));
    P_k_plus = P_k_minus - K*P_y*K';

sigma_r_z = sqrt(P_k_plus(end,end));

%}