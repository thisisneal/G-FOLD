function[] = make_video(filename, tv, r, v, u)
framerate = 10;
dt_frame = 1/framerate;
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
    pt = scatter(interp_r(1,i), interp_r(2,i), 100, 'filled');
    frame = getframe(gcf);
    writeVideo(video_writer,frame);
    delete(line);
    delete(pt);
    delete(arrow);
end
close(gcf);
close(video_writer);
