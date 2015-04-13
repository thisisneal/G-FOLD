% Neal Bhasin
% 2015-04-10
% Implementation of convex programming algorithm for powered descent guidance.
% Note: This script requires an installation of MATLAB CVX.

% Primary reference:
% Açikmese, Behçet, et al. "Enhancements on the convex programming based 
% powered descent guidance algorithm for mars landing." (2008).

% Vehicle fixed parameters
m_dry = 1505;       % Vehicle mass without fuel [kg]
Isp = 225;          % Specific impulse [s]
g0 = 9.80665;       % Standard earth gravity [m/s^2]
g = [0 ; -3.7114];  % Mars gravity vector [m/s^2]
max_throttle = 0.8; % Max open throttle [.%]
min_throttle = 0.3; % Min open throttle [.%]
T_max = 6 * 3100;   % Max total thrust force at 1.0 throttle [N]
phi = 27;           % The cant angle of thrusters [deg]

% Initial conditions
m_wet = 1905;           % Vehicle mass with fuel [kg]
r0 = [  2 ; 1.5] * 1e3; % Initial position [x;z] [m]
v0 = [100 ; -75];       % Initial velocity [x;z] [m/s]

% Target conditions
rf = [ 0 ; 0 ];
vf = [ 0 ; 0 ];

tf = 75;  % Target end time [s]
dt = 1.0; % Discrete node time interval [s]
N = (tf / dt) + 1;
tv = 0:dt:tf;

% Description: ___

% z === ln m
% u === T / m
% s === G / m

alpha = 1 / (Isp * g0 * cosd(phi));
r1 = min_throttle * T_max * cosd(phi);
r2 = max_throttle * T_max * cosd(phi);

cvx_begin
    % Parameterize trajectory position, velocity, thrust acceleration, ln mass
    variables r(2,N) v(2,N) u(2,N) z(1,N) s(1,N)
    % Maximize ln of final mass -> Minimize fuel used
    maximize( z(N) )
    
    subject to
        % Initial condition constraints
        r(:,1) == r0;
        v(:,1) == v0;
        z(1) == log(m_wet);
        % Terminal condition constraints
        r(:,N) == rf;
        v(:,N) == vf;
        % Dynamical constraints
        for i=1:N-1
            % Position / Velocity
            v(:,i+1) == v(:,i) + dt*g + (dt/2)*(u(:,i) + u(:,i+1));
            r(:,i+1) == r(:,i) + (dt/2)*(v(:,i) + v(:,i+1)) + ...
                (dt^2/12)*(u(:,i+1) - u(:,i));
            % Mass
            z(i+1) == z(i) - (alpha*dt/2)*(s(i) + s(i+1));
        end
        % Thrust limit, mass flow limit
        for i=1:N
            norm(u(:,i)) <= s(i);
            %
            z0_term = m_wet - alpha * r2 * (i-1) * dt;
            z1_term = m_wet - alpha * r1 * (i-1) * dt;
            z0 = log(z0_term);
            z1 = log(z1_term);
            mu_1 = r1 / z0_term;
            mu_2 = r2 / z0_term;
            
            s(i) >= mu_1 * (1 - (z(i) - z0)); % Linear lower bound for CVX
            %s(i) >= mu_1 * (1 - (z(i) - z0) - (1/2)*(z(i) - z0)^2);
            s(i) <= mu_2 * (1 - (z(i) - z0));
            
            z(i) >= z0;
            z(i) <= z1;
            
            % hack
            %s(i) <= r2 / z0_term;
            %s(i) >= r1 / z1_term;
        end
        % No sub-surface flight
        r(2,:) >= -1;
cvx_end

u_norms = norms(u);
m_vals = exp(z);
T_vals = u_norms .* m_vals;

% Plot results
figure; hold on;
plot(tv, r(1,:), tv, r(2,:));
title('Position (m)');

figure; hold on;
plot(tv, v(1,:), tv, v(2,:));
title('Velocity (m/s)');

figure; hold on;
plot(r(1,:), r(2,:));
quiver(r(1,:), r(2,:), u(1,:), u(2,:), .25);
title('Trajectory (m)');

figure;
plot(tv, u(1,:), tv, u(2,:));
title('Acceleration (m/s^2)');

figure; hold on;
plot(tv, T_vals);
title('Thrust (N)');
% 
% figure;
% plot(tv, u_norms);
% title('Acceleration (m/s^2)');
% 
% figure;
% plot(tv, a_dirs);
% title('Acceleration (deg)');

figure;
plot(tv, exp(z));
title('Mass (kg)');
