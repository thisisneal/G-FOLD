% command thrust
function[u] = traj_follower(G, x, t)
    mi = interp1(G.tv, G.m, t);
    u1 = interp1(G.tv, G.u(1,:), t);
    u2 = interp1(G.tv, G.u(2,:), t);
    u = mi * [u1 ; u2];
end
