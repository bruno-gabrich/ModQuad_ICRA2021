function gif_export(filename, dt, new_file)

% Capture the plot as an image
frame = getframe(gcf);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,256);

filename = strcat(['animations/', filename, '.gif']);
% Write to the GIF File
if new_file
    imwrite(imind,cm,filename,'gif', 'DelayTime', dt, 'Loopcount',inf);
else
    imwrite(imind,cm,filename,'gif','DelayTime', dt, 'WriteMode','append');
end

end