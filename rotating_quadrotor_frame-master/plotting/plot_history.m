function modquad = plot_history(modquad)

filename = input('File_name or none to not save: ', 's');
save_anim = ~isempty(filename);

if save_anim
    filename = strcat(['animations/', filename, '.avi']);
    disp(strcat(['Saving to file: ', filename]));
    video = VideoWriter(filename);
    video.FrameRate = 1/modquad.dt;
    video.Quality = 50;
    open(video);
end

figure(2);
clf;

modquad.init_final_plot = 1;


for i = 1:length(modquad.history.t)
    %% Show animation of modquad
    subplot(3,1,1:2);
    title('Animation');

    % Plot quads and boxes
    for q = 1:modquad.n
        if modquad.init_final_plot
            modquad.quads(q).cube_handle = patch;
            modquad.quads(q).quad_handle = patch;
        end

        modquad.quads(q).cube_handle = plot_cylinder(...
            modquad.history.quads(q).x_i_w(:,i),...
            modquad.history.R_0_w(:,:,i) * modquad.quads(q).R_i0_0, ...
            modquad.quads(q).cube_handle);

        modquad.quads(q).quad_handle = plot_quad(...
            modquad.history.quads(q).x_i_w(:,i), ...
            modquad.history.quads(q).R_i_w(:,:,i), ...
            modquad.quads(q).quad_handle);
        hold on;
    end

    %% Initialize modquad handles
    if modquad.init_final_plot
        modquad.all_pos_handle =       plot3(0,0,0, 'Color',[0.3010 0.7450 0.9330],'Marker','o','MarkerSize', 1);
        modquad.cur_handle =           quiver3(0,0,0,0,0,0,'Color', [0.3010 0.7450 0.9330], ...
            'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor',[0.3010 0.7450 0.9330],...
            'LineWidth', 1);

        modquad.des.all_pos_handle =   plot3(0,0,0,'Color', [1 0 1]);
        modquad.des.cur_handle =       quiver3(0,0,0,0,0,0, 'Color',[1 0 1], ...
            'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', [1 0 1],...
            'LineWidth', 1);
        modquad.forces_handle =        quiver3(0,0,0,0,0,0, 'k-');

        % These don't change, so only plot them once
        set(modquad.all_pos_handle, ...
            'XData', modquad.history.x_0_w(1,:), ...
            'YData', modquad.history.x_0_w(2,:), ...
            'ZData', modquad.history.x_0_w(3,:));

        set(modquad.des.all_pos_handle, ...
            'XData', modquad.history.des.x_0_w(1,:), ...
            'YData', modquad.history.des.x_0_w(2,:), ...
            'ZData', modquad.history.des.x_0_w(3,:));

        axis equal;

        x_min = min(modquad.history.x_0_w, [], 2) - [2 2 2]';
        x_max = max(modquad.history.x_0_w, [], 2) + [2 2 2]';

        axis([x_min(1) x_max(1) x_min(2) x_max(2) x_min(3) x_max(3)]);
        
        set(gcf, 'Position', [0 0 1000 1000]);
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
    end

    %% Update modquad handles

    axis_length = 0.5;

    set(modquad.cur_handle, ...
        'XData', repmat(modquad.history.x_0_w(1,i), 1, 3), ...
        'YData', repmat(modquad.history.x_0_w(2,i), 1, 3), ...
        'ZData', repmat(modquad.history.x_0_w(3,i), 1, 3), ...
        'UData', axis_length*modquad.history.R_0_w(1,:,i), ...
        'VData', axis_length*modquad.history.R_0_w(2,:,i), ...
        'WData', axis_length*modquad.history.R_0_w(3,:,i));

    set(modquad.des.cur_handle, ...
        'XData', repmat(modquad.history.des.x_0_w(1,i), 1, 3), ...
        'YData', repmat(modquad.history.des.x_0_w(2,i), 1, 3), ...
        'ZData', repmat(modquad.history.des.x_0_w(3,i), 1, 3), ...
        'UData', axis_length*modquad.history.des.R_0_w(1,:,i), ...
        'VData', axis_length*modquad.history.des.R_0_w(2,:,i), ...
        'WData', axis_length*modquad.history.des.R_0_w(3,:,i));

    modquad = plot_forces(modquad,i);


%     view(3);
    view(3*modquad.history.t(i), 45 - 3*modquad.history.t(i));
    grid on;

    %% Plot positions
    subplot(3,2,5);
    hold on;

    if modquad.init_final_plot
        modquad.history.x_handle = plot(modquad.history.t, modquad.history.x_0_w(1,:), 'r--');
        modquad.history.y_handle = plot(modquad.history.t, modquad.history.x_0_w(2,:), 'g--');
        modquad.history.z_handle = plot(modquad.history.t, modquad.history.x_0_w(3,:), 'b--');

        modquad.history.des.x_handle = plot(modquad.history.t, modquad.history.des.x_0_w(1,:), 'r-');
        modquad.history.des.y_handle = plot(modquad.history.t, modquad.history.des.x_0_w(2,:), 'g-');
        modquad.history.des.z_handle = plot(modquad.history.t, modquad.history.des.x_0_w(3,:), 'b-');

        modquad.x_handle = plot(modquad.history.t(i), modquad.history.x_0_w(1,i), 'ro');
        modquad.y_handle = plot(modquad.history.t(i), modquad.history.x_0_w(2,i), 'go');
        modquad.z_handle = plot(modquad.history.t(i), modquad.history.x_0_w(3,i), 'bo');
        %     axis([0 modquad.his -inf inf]);
        %     hold on;
        title('Position')
        ylabel('Position [m]');
        xlabel('Time [s]');
%         legend('x', 'y', 'z', 'xd', 'yd', 'zd');
    end

    set(modquad.x_handle, 'XData', modquad.history.t(i), 'YData', modquad.history.x_0_w(1,i));
    set(modquad.y_handle, 'XData', modquad.history.t(i), 'YData', modquad.history.x_0_w(2,i));
    set(modquad.z_handle, 'XData', modquad.history.t(i), 'YData', modquad.history.x_0_w(3,i));

    %% Plot orientations
    subplot(3,2,6);
    hold on;

    if modquad.init_final_plot
        colors = [1 0 0; 0 1 0; 0 0 1];

        for j = 1:3
            modquad.history.all_q_handle{j} = plot(...
                modquad.history.t, modquad.history.eul_0_w(j,:), ...
                '--', 'Color', colors(j,:));

            modquad.history.des.all_q_handle{j} = plot(...
                modquad.history.t, modquad.history.des.eul_0_w(j,:), ...
                '-', 'Color', colors(j,:));

        end
            modquad.history.q_tip_handle1 = plot(0,0, 'o', 'Color', colors(1,:));
            modquad.history.q_tip_handle2 = plot(0,0, 'o', 'Color', colors(2,:));
            modquad.history.q_tip_handle3 = plot(0,0, 'o', 'Color', colors(3,:));
            %modquad.history.q_tip_handle4 = plot(0,0, 'o', 'Color', colors(4,:));
    
            title('Structure Orientation');
            xlabel('Time [s]');
            ylabel('Euler Angles [^{\circ}]');
%             legend('q1', 'q2', 'q3', 'q4', 'q1d','q2d','q3d','q4d');
    
    end

        set(modquad.history.q_tip_handle1, 'XData', modquad.history.t(i), 'YData', modquad.history.eul_0_w(1,i));
        set(modquad.history.q_tip_handle2, 'XData', modquad.history.t(i), 'YData', modquad.history.eul_0_w(2,i));
        set(modquad.history.q_tip_handle3, 'XData', modquad.history.t(i), 'YData', modquad.history.eul_0_w(3,i));
        %set(modquad.history.q_tip_handle4, 'XData', modquad.history.t(i), 'YData', modquad.history.q_0_w(4,i));
    modquad.init_final_plot = 0;
    %def = legend('desired roll','roll','desired pitch','pitch','desired yaw','yaw');
    %def.FontSize = 5;
    ylim([-30 30])
    drawnow();
    if save_anim
        writeVideo(video, getframe(gcf));
    end
%     pause(.03);
end
if save_anim
    close(video);
end
close all;
end
