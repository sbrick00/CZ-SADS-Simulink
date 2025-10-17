clc; clear
load("serial_13-Jun-2025.mat")

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


t_start = 0;
start_idx = find(t > t_start, 1, 'first');
t_0 = t(start_idx);

t_plt = t(start_idx:end)-t_0;
eulerAngles_plt = deg2rad(eulerAngles(start_idx:end,1:3));
bodyRates_plt = bodyRates(start_idx:end,1:3);
mmu_pos_plt = (1/3200).*.010.*(stepperPos(start_idx:end,:)-stepperPos(start_idx,:));

quat = eul2quat(deg2rad(eulerAngles_plt),'XYZ');
g_b = zeros(length(t_plt),3);
Pp = zeros(length(t_plt),3,3);
omega_p = zeros(length(t_plt),3);

for i = 1:length(t_plt)
    phi = (eulerAngles_plt(i,1)); % roll
    theta = (eulerAngles_plt(i,2)); % pitch
    psi = (eulerAngles_plt(i,3)); % yaw;
    g_b(i,:) = [-9.81*sin(theta), 9.81*sin(phi)*cos(theta), -9.81*cos(phi)*cos(theta) ];
    % g_b(i,:) = eul2rotm([phi,theta,psi],'XYZ') * [0 0 -9.81]';
    g_b = g_b(i,:);
    Pp = eye(3) - (g_b*g_b')./(norm(g_b)^2)
    omega_p(i,:) = Pp*bodyRates_plt(i,:)';
end

eps = quat(:,2:4);
% P = 

% figure
% plot(t_plt,g_b)
%%
f = gen_tiled_layout(3);
tiles = tiledlayout(f,3,1,'Padding','compact','TileSpacing','compact');

ax1 = nexttile;
plot(t_plt,eulerAngles_plt(:,1:2),'LineWidth',1.5)
grid on

ylabel("Euler Angles [rad]")
legend("$\phi$","$\theta$")


ax2 = nexttile;
plot(t_plt,bodyRates_plt(:,1:2),'LineWidth',1.5)
% plot(t_plt,omega_p,'LineWidth',1.5)
grid on

ylabel("Body Rates [rad/s]")
legend("$\omega_x$","$\omega_y$")


ax3 = nexttile;
plot(t_plt,mmu_pos_plt,'LineWidth',1.5)
grid on
ylabel("Sliding Mass Positions [m]") 
legend("$\Delta\,d_x$","$\Delta\,d_y$")
xlabel("Time [s]")
fontsize(13,'points')
exportgraphics(f,'C:\Users\camer\OneDrive\Desktop\CZ_Thesis_Latex\plots\adaptive_hardware_failure.pdf','ContentType','vector');        % axes only