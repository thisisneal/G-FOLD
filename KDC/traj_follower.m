% command thrust
function[u] = traj_follower(G, x, t)
    % Introduce simulated nav error
    pos_sigma = 1.0;
    vel_sigma = 0.2;
    m_sigma = 0.5;
    r = x(1:2) + normrnd(0, pos_sigma, 2, 1);
    v = x(3:4) + normrnd(0, vel_sigma, 2, 1);
    m = x(5) + normrnd(0, m_sigma, 1, 1);
    % Feed-forward acceleration commands linear interpolation
    a_ff = fnval(G.a_spline, t);
    m_ff = fnval(G.m_spline, t);
    u_ff = a_ff .* m_ff;
    % PD control on position, velocity multiplied by current mass
    r_d = fnval(G.r_spline, t);
    v_d = fnval(G.v_spline, t);
    Kp = 0.1;
    Kd = 0.5;
    u_c = m * (Kp * (r_d - r) + Kd * (v_d - v));
    u = u_ff + u_c;
    % Enforce thrust limits (relax 1% for control)
    u_mag = norm(u);
    if(u_mag == 0)
        u = [0 ; G.r1];
    elseif(u_mag > G.r2 * 1.01)
        u = G.r2 * u / u_mag;
    elseif(u_mag < G.r1 * 0.99)
        u = G.r1 * u / u_mag;
    end    
end
