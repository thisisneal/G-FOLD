%

% Vehicle fixed parameters
m_dry = 1505;
Isp = 225;
g = [0 ; -3.7114];
T_max = 6 * 3100;

% Initial conditions
m_wet = 1905;
r0 = [  2 ; 1.5] * 1e3;
v0 = [100 ; -75];

% Target conditions
rf = [ 0 ; 0 ];
vf = [ 0 ; 0];

N = tf + 1; % 1 hz discrete time nodes
tf = 72;
dt = tf / (N - 1);
tv = 0:dt:N;

cvx_begin
    variables r(2,N) T(2,N) m(1,N)
    
    maximize( m(1,N) )
    
    subject to
        % Initial condition constraints
        m(1) == m_wet;
        r(:,1) == r0;
        r(:,2) == r0 + v0 * dt;
        % Terminal condition constraints
        r(:,N)   == rf;
        r(:,N-1) == rf - vf * dt;
        % Dynamical constraints
        for i=3:N
            % Acceleration
            %(r(:,i) - 2*r(:,i-1) + r(:,i-2))/dt^2 == g + T(:,i) / m(i);
            (r(:,i) - 2*r(:,i-1) + r(:,i-2))/dt^2 == g + T(:,i) / m_wet;
            m(i) <= m(i-1);
        end
cvx_end

figure;
plot(tv, r(1,:), tv, r(2,:));

figure;
plot(tv, m);
