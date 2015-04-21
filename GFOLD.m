% Neal Bhasin
% 2015-04-21
% G-FOLD outer time-optimization routine

% Vehicle/planet parameters p must include:
%  p.phi ; p.T_max ; p.max_throttle ; p.min_throttle ; p.Isp ; p.m_dry ; p.g
function [m_used, r, v, u, m] = GFOLD(dt, r0, v0, rf, vf, m_wet, p)
    g0 = 9.80665; % Standard earth gravity [m/s^2]
    alpha = 1 / (p.Isp * g0 * cosd(p.phi));
    r1 = p.min_throttle * p.T_max * cosd(p.phi);
    r2 = p.max_throttle * p.T_max * cosd(p.phi);
    t_min = p.m_dry * norm(vf - v0) / r2
    t_max = (m_wet - p.m_dry) / (alpha * r1)
    
    cvx_solver SEDUMI
    obj_fun = @(t)( GFOLD_fix_time(round(t), dt, r0, v0, rf, vf, m_wet, p) );
    options = optimset('TolX',0.1,'Display','iter');
    t_opt = fminbnd(obj_fun, t_min, t_max, options);
end