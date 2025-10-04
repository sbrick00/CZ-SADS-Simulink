clc
clear
% ORIGINAL CHOI VALUES
% m_s = 134;
% J_0 = [20.2012 -0.0899   -0.0469;
%       -0.0899   12.9943  -0.4187;
%      -0.0469  -0.4187    16.276;];

% r_0 = [-5*10^-6 5*10^-6 -3*10^-4]';

% SADS Values


m_s = 29; % [kg]
J_0 = [1.815  -0.014 0.004;
      -0.014  1.348  0.008;
       0.004  0.008 1.475];

stepsPerRev = 3200;

r_0 = [4*10^-3, 2*10^-3 ,-2*10^-2]'; % solveable condition (displaced battery)
% r_0 = [0.1*10^-3, 0.1*10^-3 ,-1*10^-3]'; % simple rocking motion
% r_0 = [0 0 0]'; % no inbalance

omega_0 = [0 0.00 0.06]'; % slight spin, helps with stability
% omega_0 = [0 0.00 0.00]';

EA_0 = deg2rad([1 0 45]'); 

% transformations are inertial to body
% XYZ rotation order must be explicitly specified
q_0 = eul2quat(EA_0',"XYZ")'; 
q_d = [1 0 0 0]';

q_e_0 = quatmultiply(q_d',q_0');

q_e_0 = [q_e_0(2) q_e_0(3) q_e_0(4)]';

g_N = [0 0 -9.81];

% six screws + n plates + ballscrew
n = 4;
m_mmu = 2*((0.015*5) + (6*.193) + 0.177);

% idea 1: higher gain scale, all gains similar
gainScale = 0.5;
K_omega = gainScale*1;
K_q = gainScale*1;
K_I = gainScale*5;

% idea 2: lower gain scale, high integral gain
% gainScale = 0.1;
% K_omega = gainScale*5;
% K_q = gainScale*5;
% K_I = gainScale*5;

% gainScale = 0; % to test tumbling motion

t_sim = 90;


% constants for testing adaptive control
k_p = 0.001; 
r_hat_0 = [0 0 -0.1]'; % initial estimation of r
%%
out = sim('Choi_Simulink.slx');
%%
clc
t = out.tout;
r_b = squeeze(out.r_b.signals.values);
r_mmus = out.r_mmus.signals.values;
EA_b_N = out.EA_b_N.signals.values;
q_b_N = out.q_b_N.signals.values;

figure;
plot(t,r_b,'LineWidth',1.2);
grid on;
legend("r_x","r_y","r_z")
xlabel("Time [s]")
ylabel("Center of Mass Components in Body Frame")

figure;
plot(t,r_mmus,'LineWidth',1.2)
hold on
grid on;
yline(0.087,'--','LineWidth',1.2)
yline(-0.087,'--','LineWidth',1.2)
xlabel("Time [s]")
ylabel("Sliding Mass Positions")
legend("x Mass","y Mass", "z Mass", "Travel Limit")

figure;
% plot(t,EA_b_N,'LineWidth',1.2)
plot(t,rad2deg(quat2eul(q_b_N,'XYZ')),'LineWidth',1.2)
grid on
legend("Roll","Pitch","Yaw")

% figure;
% stepper_speeds_x = gradient(r_mmus(:,1),t);
% stepper_speeds_y = gradient(r_mmus(:,2),t);
% stepper_speeds_x = (stepper_speeds_x./0.010).*stepsPerRev;
% stepper_speeds_y = (stepper_speeds_y./0.010).*stepsPerRev;
% plot(t,stepper_speeds_x,'LineWidth',1.2)
% hold on 
% plot(t,stepper_speeds_y,'LineWidth',1.2)
% title("Stepper Motor Speeds")
% grid on
% ylabel("Steps/sec")
% 
% figure;
% stepper_accels_x = gradient(stepper_speeds_x,t);
% stepper_accels_y = gradient(stepper_speeds_y,t);
% plot(t,stepper_accels_x,'LineWidth',1.2)
% hold on
% plot(t,stepper_accels_y);
% title("Stepper Motor Accelerations")
% grid on
% ylabel("Steps/sec^2")
% ylim([-5000 5000])

%%
% n = length(out.tout);
% C_b_N = quat2dcm(q_b_N);
% x_N = zeros(n,3);
% y_N = zeros(n,3);
% z_N = zeros(n,3);
% 
% for i = 1:n
%     C_N_b = C_b_N(:,:,i)';
%     x_N(i,:) = C_N_b*[1 0 0]';
%     y_N(i,:) = C_N_b*[0 1 0]';
%     z_N(i,:) = C_N_b*[0 0 1]';
% end
% 
% figure;
% axis equal;
% grid on;
% hold on;
% 
% % Set axis limits (adjust based on your data)
% scale = 1;
% sliderScale = 0.25;
% xlim([-1.3*scale, 1.3*scale]);
% ylim([-1.3*scale, 1.3*scale]);
% zlim([-1.3*scale, 1.3*scale]);
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Moving Vector Animation');
% 
% % Camera settings
% view(3);                     % Default 3D view
% campos([5, 5, 5]);           % Camera position (adjust as needed)
% camtarget([0, 0, 0]);        % Camera looks at the origin
%            % Z-axis is the "up" direction
% 
% quiver3(0, 0, 0, scale/2, 0, 0, 'b', 'LineWidth', 1, 'MaxHeadSize', 0.5); % X-axis (red)
% quiver3(0, 0, 0, 0, scale/2, 0, 'b', 'LineWidth', 1, 'MaxHeadSize', 0.5); % Y-axis (green)
% quiver3(0, 0, 0, 0, 0, scale/2, 'b', 'LineWidth', 1, 'MaxHeadSize', 0.5); % Z-axis (blue)
% 
% % COM
% hVector1 = quiver3(0, 0, 0, 0, 0, 0, 'LineWidth', 2, 'MaxHeadSize', 0.2, 'Color', 'g');
% 
% % body basis vectors
% hVector2 = quiver3(0, 0, 0, 0, 0, 0, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'r');
% hVector3 = quiver3(0, 0, 0, 0, 0, 0, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'r');
% hVector4 = quiver3(0, 0, 0, 0, 0, 0, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'r');
% 
% for i = 1:n-1
% 
%     % COM
%     % hVector1.UData = r_N(i, 1);
%     % hVector1.VData = r_N(i, 2);
%     % hVector1.WData = r_N(i, 3);
% 
%     % x body
%     hVector2.UData = scale*x_N(i, 1);
%     hVector2.VData = scale*x_N(i, 2);
%     hVector2.WData = scale*x_N(i, 3);
% 
%     % y body
%     hVector3.UData = scale*y_N(i, 1);
%     hVector3.VData = scale*y_N(i, 2);
%     hVector3.WData = scale*y_N(i, 3);
% 
%     % z body
%     hVector4.UData = scale*z_N(i, 1);
%     hVector4.VData = scale*z_N(i, 2);
%     hVector4.WData = scale*z_N(i, 3);
% 
%     % Pause for a short duration to create animation effect
%     pause((out.tout(i+1)-out.tout(i)))
%     disp(out.tout(i))
% end

%%
clc
EA_0 = deg2rad([-2.83 -9.89 149]'); 

% transformations are inertial to body
% XYZ rotation order must be explicitly specified
q_0 = eul2quat(EA_0',"XYZ")'
