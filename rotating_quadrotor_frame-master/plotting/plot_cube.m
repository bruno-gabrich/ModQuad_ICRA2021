%% Function to plot a cube mesh at a given location and size
function patch_handle = plot_cube(center, size, rot, patch_handle)
center = reshape(center, 3,1);
size = reshape(size, 3,1);

pts = [...
    -1 -1 -1;
    1 -1 -1;
    1 1 -1;
    -1 1 -1;
    -1 -1 1;
    1 -1 1;
    1 1 1;
    -1 1 1]' ./2 ;
pts = rot*(pts.* repmat(size,1,8)) + repmat(center,1,8);

faces = [...
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    4 3 7 8;
    1 4 8 5;
    2 3 7 6];

patch_handle.Vertices = pts';
patch_handle.Faces = faces;
patch_handle.FaceColor = [0 0 1];
patch_handle.FaceAlpha = .1;


% axis equal;
% axis(repmat([-3 3], 1,3));
% view(3);
grid on;
end
