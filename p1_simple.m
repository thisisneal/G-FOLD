% [x ; z]

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
dt = 1; % 1 hz discrete time nodes
N = (tf / dt) + 1;
tv = 0:dt:tf;

cvx_begin
    variables r(2,N) a(2,N)
    
    minimize( norm(a) )
    
    subject to
        % Initial condition constraints
        r(:,1) == r0;
        r(:,2) == r0 + v0 * dt;
        % Terminal condition constraints
        r(:,N)   == rf;
        r(:,N-1) == rf - vf * dt;
        % Dynamical constraints
        for i=3:N
            % Acceleration
            (r(:,i) - 2*r(:,i-1) + r(:,i-2))/dt^2 == g + a(:,i);
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

figure;
plot(tv, r(1,:), tv, r(2,:));
title('Position (m)');

figure;
plot(r(1,:), r(2,:));
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
