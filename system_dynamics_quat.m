function [xp] = system_dynamics_quat(x, u, L)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
include_namespace_dq

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

%% Force body frame Drone

f = [0; 0; u(1)];
Tau = [u(2); u(3); u(4)];

%% Velocity of the system

vec_q_dual = x(1:8);
vec_xi_dual = x(9:16);

%% Quadternio dual
unit_q_dual = normalize(DQ(vec_q_dual));
xi_dual = DQ(vec_xi_dual);


q_dual_p = (1/2)*(unit_q_dual*xi_dual);


p = vec3(translation(unit_q_dual));

omega = vec3(P(xi_dual));

p_p= vec3(D(xi_dual))-cross(omega, p);

a = cross(-inv(I)*omega,I*omega); 

F_dual = a + E_*(cross(a,p)+cross(omega, p_p));
U_dual = pinv(I)*Tau + E_*((f/m)+cross(pinv(I)*Tau,p));


xi_dual_p = F_dual + U_dual;

%% General vector of the system
xp = zeros(16, 1);
xp(1:8) = vec8(q_dual_p);
xp(9:16) = vec8(xi_dual_p);



end

