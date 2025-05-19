function [xp] = system_dynamics(h, u, L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% System parameters
g = L(1);
m = L(2);
Jxx = L(3);
Jyy = L(4);
Jzz = L(5);

%% Inertia Mmatrix
I = [Jxx, 0 0;...
     0, Jyy, 0;...
     0, 0, Jzz];
 
%% Torques inputs

e = [0;0;1];

%% Velocity of the system
pos = h(1:3);
v = h(4:6);
quat = h(7:10);
omega = h(11:13);

%% Rotational matrix
[R] = quaternionToRotationMatrix(quat);

%% Force body frame Drone

T = [0; 0; u(1)];
Tau = [u(2); u(3); u(4)];

%% Aceleration system 
v_p = -e*g + (R * T)/m;
q_dot1 = quat_dot(quat, omega);
q_dot = quat_p(quat, omega);



error = quaternionError(q_dot1, q_dot);

disp(['Error between quaternions: ', num2str(error)]);

omega_dot = inv(I)*(Tau - cross(omega, I*omega));

%% General vector of the system
xp = zeros(13, 1);
xp(1:3) = v;
xp(4:6) = v_p;
xp(7:10) = q_dot;
xp(11:13) = omega_dot;
end

