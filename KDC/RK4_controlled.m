% Neal Bhasin
% 2015-04-26
% Runge-Kutta 4 integration with controller
%
% Inputs:
% x_0 : Initial state (Nx1)
% dt  : Simulation time step (s)
% tf  : Final time (s)
% control_rate : Controller frequency (Hz)
% ode_fun      : Time-invariant continuous time dynamics function handle, x_dot = ode_fun(x, u)
% control_fun  : Time-dependent controller function handle, u = control_fun(x, t)
%
% Outputs:
% xs : States   (NxM)
% us : Controls (NxM)
% tv : Times    (1xM)
function [xs, us, tv] = RK4_controlled(x_0, dt, tf, control_rate, ode_fun, control_fun)
    N = 1 + (tf / dt);
    control_ticks = ceil((1 / control_rate) / dt);
    x = x_0;
    u = control_fun(x_0, 0);
    xs = zeros(size(x_0, 1), N);
    us = zeros(size(u, 1), N);
    tv = 0:dt:tf;
    xs(:,1) = x;
    us(:,1) = u;
    for i=2:N
        k1 = dt * ode_fun(x,        u);
        k2 = dt * ode_fun(x + k1/2, u);
        k3 = dt * ode_fun(x + k2/2, u);
        k4 = dt * ode_fun(x + k3,   u);
        x = x + (1 / 6) * (k1 + 2*k2 + 2*k3 + k4);
        % Update control input at specified frequency
        if mod(i, control_ticks) == 0
            u = control_fun(x, tv(i));
        end
        xs(:,i) = x;
        us(:,i) = u;
    end
end
