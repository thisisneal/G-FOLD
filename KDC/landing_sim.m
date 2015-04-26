% Simulate landing
r_0 = [2 ; 1.5]*1e3;
v_0 = [100 ; -75];
m_0 = 1905;
x_0 = [r_0; v_0 ; m_0];

r_d = [0 ; 0];
v_d = [0 ; 0];

p.phi = 27;
p.T_max = 6 * 3100;
p.max_throttle = 0.8; p.min_throttle = 0.3; 
p.Isp = 225;
p.m_dry = 1505;
p.g = [0 ; -3.7114];

dt = 0.01;
tf = 74.5;

% Initialize dynamics with vehicle/planet parameters
ode_fun = @(x,u)(dynamics_2D(p,x,u));
% Initialize controller with GFOLD-computed trajectory
%%%[G.tv, ~, G.r, G.v, G.u, G.m] = GFOLD(50, r_0, v_0, r_d, v_d, m_0, 180, p);
control_fun = @(x, t)(traj_follower(G, x, t));
control_rate = 20;

[xs, us, tv] = RK4_controlled(x_0, dt, tf, control_rate, ode_fun, control_fun);
r = xs(1:2,:);
v = xs(3:4,:);
m = xs(5,:);
a = [us(1,:) ./ m ; us(2,:) ./ m];
plot_run2D(tv, r, v, a, m);
