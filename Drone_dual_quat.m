%% Code to simulate Aerial vehicle in quadternions
%% Init time
clc, clear all, close all;

include_namespace_dq
%% Set time parameters
frecuencia = 30;
ts = 1/frecuencia;
t_final = 10;
t = (0:ts:t_final);

%% Initial state

p_init = [0;  0; 0]; %% Initial position of the system
p_p= [0; 0; 0]; %% Initial Velocity of the system
omega = [0; 0; 0]; %%


q = [0.8799807;  -0.3358824; 0.3358824; 0];
p = [0;p_init];


q_dual = q + 1/2* E_* (quaternionMultiply(q, p));
xi_dual = [0;omega] + E_*[0; p_p + cross(omega,p_init)];

vec_q_dual = vec8(q_dual);
vec_xi_dual = vec8(xi_dual);


%% Initial Rotational Matrix
R = zeros(3,3,length(t)+1);
quat_init = q;
R(:, :, 1) = quaternionToRotationMatrix(quat_init);

%% Initial vector State
x = zeros(16, length(t) +1);
x(:, 1) = [vec_q_dual;vec_xi_dual];

%% System parameters
g = 9.80;
factor = 10;
m_drone = 0.33*factor;
Jxx_drone = (1.395e-4)*factor;
Jyy_drone = (1.395e-4)*factor;
Jzz_drone = (2.173e-4)*factor;
%% Vector of system Parametes
L_drone = [g; m_drone; Jxx_drone; Jyy_drone; Jzz_drone];

%% Control vector
u = zeros(4, length(t));

%% Simulation system
for k = 1:length(t)
    %% Controller section
    
    if k < 150
        u(:, k) = [0.5; 0.000; -0.000; 0.000];
    else
        u(:, k) = [0; 0.000; 0.000; 0.000];
    end

    
    %% System evolution
    x(:, k+1) = system_simulation_quat(x(:, k), u(:, k), L_drone, ts);
    
    unit_q_dual = normalize(DQ(x(1:8, k+1)));
    
    pose(:,k) = vec3(translation(unit_q_dual));
    q(:,k) = vec4(P(unit_q_dual));
    
    R(:, :, k+1) = quaternionToRotationMatrix(q(:,k));
    
   
 
end

disp("Test")

%% Simulation System
close all; paso=5; 
%a) Parámetros del cuadro de animación
figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 8 3]);
luz = light;
luz.Color=[0.65,0.65,0.65];
luz.Style = 'infinite';
Drone_Parameters(0.02);
G2=Drone_Plot_Rot_3D(pose(1,1),pose(2,1),pose(3,1),R(:, :, 1));hold on
plot3(pose(1,1),pose(2,1),pose(3,1),'--','Color',[56,171,217]/255,'linewidth',1.5);hold on,grid on

%axis([-9.5 9.5 -9.5 9.5 -9.5 9.5]);
 view(20,15);
for k = 1:paso:length(t)-1
    drawnow
    delete(G2);
    G2=Drone_Plot_Rot_3D(pose(1,k),pose(2,k),pose(3,k),R(:, :, k));hold on
    plot3(pose(1,1:k),pose(2,1:k),pose(3,1:k),'--','Color',[56,171,217]/255,'linewidth',1.5);
    legend({'$\mathbf{h}$','$\mathbf{h}_{des}$'},'Interpreter','latex','FontSize',11,'Location','northwest','Orientation','horizontal');
    legend('boxoff')
    title('$\textrm{Movement Executed by the Aerial Robot}$','Interpreter','latex','FontSize',11);
    xlabel('$\textrm{X}[m]$','Interpreter','latex','FontSize',9); ylabel('$\textrm{Y}[m]$','Interpreter','latex','FontSize',9);zlabel('$\textrm{Z}[m]$','Interpreter','latex','FontSize',9);
    
end

%% System pictures

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
plot(t,euler(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,euler(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,euler_d(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,x(6,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,hd(1,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t,hd(2,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,hd_d(4,1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,hd_d(3,1:length(t)),'--','Color',[26,115,160]/255,'linewidth',1); hold on

grid on;
legend({'$\phi$','$\theta$','$\psi$','$\dot{z}$','$\phi_d$','$\theta_d$','$\psi_d$','$\dot{z}_d$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,2,1)
plot(t,u(1,1:length(t)),'Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$f$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);
subplot(1,2,2)
plot(t,u(2,1:length(t)),'Color',[46,188,89]/255,'linewidth',1); hold on
plot(t,u(3,1:length(t)),'Color',[26,115,160]/255,'linewidth',1); hold on
plot(t,u(4,1:length(t)),'Color',[26,50,160]/255,'linewidth',1); hold on
grid on;
legend({'$\tau_{\phi}$','$\tau_{\theta}$','$\tau_{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Positions}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlabel('$\textrm{Time}[s]$','Interpreter','latex','FontSize',9);


