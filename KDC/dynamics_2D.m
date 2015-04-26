% MSL Powered Descent 2D Dynamics
% p is parameters ( see GFOLD.m )
function[x_dot] = dynamics_2D(p, x, u)
% Unpack state
%r = x(1:2);
v = x(3:4);
m = x(5);
%
a_thrust = u ./ m;
thrust_noise_mag = 100; % N 1-sigma
u_noise = normrnd(0, thrust_noise_mag, 2, 1);
r_dot = v;
v_dot = a_thrust + (u_noise./m) + p.g;
g0 = 9.80665; % Standard earth gravity [m/s^2]
alpha = 1 / (p.Isp * g0 * cosd(p.phi));
m_dot = -alpha * norm(u);
% Pack continuous state derivative
x_dot = [r_dot ; v_dot ; m_dot];
end
