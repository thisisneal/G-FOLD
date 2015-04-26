function[] = make_video(filename, tv, r, v, u)
framerate = 20;
speedup_factor = 2;
dt_frame = speedup_factor/framerate;
tv_frame = 0:dt_frame:tv(end);

sp_r = spapi(5, tv, r);
interp_r = fnval(sp_r, tv_frame);
sp_u = spapi(2, tv, u);
interp_u = fnval(sp_u, tv_frame);

video_writer = VideoWriter(filename);
video_writer.FrameRate = framerate;
open(video_writer);

figure; hold on;
axis([-100 3500 -100 1600]);
rectangle('Position', [-100 -100 3600 100], 'FaceColor', [0.5 0.1 0.1]);
ylabel('Altitude [m]');
xlabel('Position [m]');

for i=1:size(interp_r,2)
    ax = gca;
    ax.ColorOrderIndex = 1;
    line = plot(interp_r(1,1:i), interp_r(2,1:i), 'LineWidth', 1);
    arrow = quiver(interp_r(1,i), interp_r(2,i), -interp_u(1,i), -interp_u(2,i), 15, 'LineWidth', 2);
    % Lander triangle
    w = 60; h = 60;
    body_pts = [-w/2 0 w/2 0 ; 0 h 0 0 ];
    t = -90 + atan2d(interp_u(2,i), interp_u(1,i));
    R = [cosd(t) -sind(t) ; sind(t) cosd(t)];
    trans_x = ones(1,size(body_pts,2)) * interp_r(1,i);
    trans_y = ones(1,size(body_pts,2)) * interp_r(2,i);
    world_pts = R * body_pts + [trans_x ; trans_y];
    lander = patch(world_pts(1,:), world_pts(2,:), ax.ColorOrder(3,:));
    % Write to video, clear plots
    frame = getframe(gcf);
    writeVideo(video_writer,frame);
    delete(line);
    delete(lander);
    delete(arrow);
end
close(gcf);
close(video_writer);
