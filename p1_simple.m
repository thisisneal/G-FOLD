% Acikmese, Behcet, and Scott R. Ploen. "Convex programming approach to 
% powered descent guidance for mars landing." 
% Journal of Guidance, Control, and Dynamics 30.5 (2007): 1353-1366.

% Vehicle fixed parameters
m_dry = 1505;
Isp = 225;
g0 = 9.80665; % Standard earth gravity
g = [0 ; -3.7114]; % Mars gravity vector
max_throttle = 0.8;
T_max = 6 * 3100 * max_throttle;
phi = 27; % The cant angle of thrusters in degrees

% Initial conditions
m_wet = 1905;
r0 = [  2 ; 1.5] * 1e3;
v0 = [100 ; -75];

% Target conditions
rf = [ 0 ; 0 ];
vf = [ 0 ; 0];

tf = 75;
dt = 3.0; % 1 hz discrete time nodes
N = (tf / dt) + 1;
tv = 0:dt:tf;

cvx_begin
    variables r(2,N) v(2,N) a(2,N)
    
    minimize( norm(a) )
    
    subject to
        % Initial condition constraints
        r(:,1) == r0;
        v(:,1) == v0;
        % Terminal condition constraints
        r(:,N) == rf;
        v(:,N) == vf;
        % Dynamical constraints
        for i=1:N-1
            v(:,i+1) == v(:,i) + dt*g + (1/2)*dt*(a(:,i) + a(:,i+1));
            r(:,i+1) == r(:,i) + dt*v(:,i) + (1/6)*dt^2*(2*(a(:,i)+g) + (a(:,i+1)+g));
        end
        % Acceleration/Thrust limit
        for i=1:N
            norm(a(:,i)) <= T_max / m_wet; % Very conservative 
        end
        % No sub-surface flight
        r(2,:) >= -1;
cvx_end

a_norms = norms(a);
a_dirs = rad2deg(atan2(a(2,:), a(1,:)));

% Compute vehicle mass over time
m = zeros(1, N);
m(1) = m_wet;
for i=1:N-1
    T_i = a_norms(i) * m(i);
    m_dot = -T_i / (Isp * g0 * cosd(phi));
    m(i+1) = m(i) + m_dot * dt;
end

figure; hold on;
plot(tv, r(1,:), tv, r(2,:));
title('Position (m)');

figure; hold on;
plot(tv, v(1,:), tv, v(2,:));
title('Velocity (m/s)');

figure; hold on;
plot(r(1,:), r(2,:));
quiver(r(1,:), r(2,:), a(1,:), a(2,:), .25);
title('Trajectory (m)');

figure;
plot(tv, a(1,:), tv, a(2,:));
title('Acceleration (m/s^2)');

figure;
plot(tv, a_norms);
title('Acceleration (m/s^2)');

figure;
plot(tv, a_dirs);
title('Acceleration (deg)');

figure;
plot(tv, m);
title('Mass (kg)');
