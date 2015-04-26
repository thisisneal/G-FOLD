% MSL Powered Descent 2D Dynamics
% p is parameters ( see GFOLD.m )
function[x_dot] = dynamics_2D(p, x, u)

%r = x(1:2);
v = x(3:4);
m = x(5);

a_thrust = u ./ m;
r_dot = v;
v_dot = a_thrust + p.g;
g0 = 9.80665; % Standard earth gravity [m/s^2]
alpha = 1 / (p.Isp * g0 * cosd(p.phi));
m_dot = -alpha * norm(u);

x_dot = [r_dot ; v_dot ; m_dot];

end
