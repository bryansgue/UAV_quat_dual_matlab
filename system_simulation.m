function [x] = system_simulation(x, u, L, ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = system_dynamics(x, u, L);
k2 = system_dynamics(x + ts/2*k1, u, L); % new
k3 = system_dynamics(x + ts/2*k2, u, L); % new
k4 = system_dynamics(x + ts*k3, u, L); % new

x = x +ts/6*(k1 +2*k2 +2*k3 +k4); % new

end