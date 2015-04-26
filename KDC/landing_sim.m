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
tf = 75.0;

ode_fun = @(x,u)(dynamics_2D(p,x,u));
control_fun = @(x)(zeros(2,1));
control_rate = 20;

[xs, us, tv] = RK4_controlled(x_0, dt, tf, control_rate, ode_fun, control_fun);
r = xs(1:2,:);
v = xs(3:4,:);
m = xs(5,:);
plot_run2D(tv, r, v, us, m);
