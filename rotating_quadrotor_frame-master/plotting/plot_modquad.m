function modquad = plot_modquad(modquad)
figure(1);

%% 3d animation
% subplot(2,1,1);

for i = 1:modquad.n
    if modquad.init_plot
        modquad.quads(i).cube_handle = patch;
        modquad.quads(i).quad_handle = patch;
    end

    modquad.quads(i).cube_handle = plot_cylinder(...
        modquad.quads(i).x_i_w, modquad.R_0_w*modquad.quads(i).R_i0_0, modquad.quads(i).cube_handle);
    modquad.quads(i).quad_handle = plot_quad(...
        modquad.quads(i).x_i_w, modquad.quads(i).R_i_w, modquad.quads(i).quad_handle);
    hold on;
end

if modquad.init_plot
    modquad.pos_handle = plot3(modquad.x_0_w(1), modquad.x_0_w(2), modquad.x_0_w(3), 'b-');
    modquad.pos_tip_handle = plot3(modquad.x_0_w(1), modquad.x_0_w(2), modquad.x_0_w(3), 'bo');
    modquad.R_handle = quiver3(zeros(3,1), zeros(3,1), zeros(3,1), ones(3,1), ones(3,1), ones(3,1), 'b');

    modquad.des.pos_handle = plot3(modquad.des.x_0_w(1), modquad.des.x_0_w(2), modquad.des.x_0_w(3), 'r-');
%     modquad.des.pos_tip_handle = plot3(modquad.des.x_0_w(1), modquad.des.x_0_w(2), modquad.des.x_0_w(3), 'ro');
    modquad.des.pos_tip_handle = quiver3(zeros(3,1), zeros(3,1), zeros(3,1), ones(3,1), ones(3,1), ones(3,1), 'r');


    modquad.forces_handle = quiver3(1:10, 1:10, 1:10, 1:10);
    axis equal;
        axis(repmat([-5 5], 1,3));
%     axis([-3 3 -3 3 -3 3]);
    view(3);
    set(gcf, 'Position', [0 0 1000 1000]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end
set(modquad.pos_handle, 'XData', modquad.history.x_0_w(1,:), ...
    'YData', modquad.history.x_0_w(2,:), ...
    'ZData', modquad.history.x_0_w(3,:));

set(modquad.pos_tip_handle, ...
    'XData', modquad.x_0_w(1), ...
    'YData', modquad.x_0_w(2), ...
    'ZData', modquad.x_0_w(3));

axis_length = 0.5;

set(modquad.R_handle, ...
    'XData', repmat(modquad.x_0_w(1), 1, 3), ...
    'YData', repmat(modquad.x_0_w(2), 1, 3), ...
    'ZData', repmat(modquad.x_0_w(3), 1, 3), ...
    'UData', axis_length*modquad.R_0_w(1,:), ...
    'VData', axis_length*modquad.R_0_w(2,:), ...
    'WData', axis_length*modquad.R_0_w(3,:));

set(modquad.des.pos_tip_handle, ...
    'XData', repmat(modquad.des.x_0_w(1,end), 1, 3), ...
    'YData', repmat(modquad.des.x_0_w(2,end), 1, 3), ...
    'ZData', repmat(modquad.des.x_0_w(3,end), 1, 3), ...
    'UData', axis_length*modquad.des.R_0_w(1,:), ...
    'VData', axis_length*modquad.des.R_0_w(2,:), ...
    'WData', axis_length*modquad.des.R_0_w(3,:));

set(modquad.des.pos_handle, ...
    'XData', modquad.history.des.x_0_w(1,:), ...
    'YData', modquad.history.des.x_0_w(2,:), ...
    'ZData', modquad.history.des.x_0_w(3,:));
modquad = plot_forces(modquad);
% view(45+5*modquad.t, 30)


% view(3);
grid on;

% subplot(2,1,2);
% if modquad.init_plot
%     modquad.x_0_wx_handle = plot(modquad.history.t, modquad.history.q_0_w(1,:));
%     axis([0 10 -inf inf]);
%     hold on;
%
% end
%
% set(modquad.x_0_wx_handle, 'XData', modquad.history.t, 'YData', modquad.history.q_0_w(1,:));

modquad.init_plot = 0;
end
