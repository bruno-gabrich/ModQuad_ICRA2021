%% Function to plot a cube mesh at a given location and size
function patch_handle = plot_cylinder(center, rot, patch_handle)
center = reshape(center, 3,1);


center = reshape(center, 3,1);

arm_length = .3;
box_length = .5; % half
prop_radius = 0.3;
circle_res = 20;

theta = 0:(2*pi/(circle_res-1)):2*pi;

base_circle = [cos(theta); sin(theta); zeros(1, circle_res)];
circle_top = prop_radius * base_circle;
circle_bottom = prop_radius * base_circle -[0;0;0.05];




pts = [circle_top,circle_bottom];  
pts = rot*(pts) + repmat(center,1,size(pts,2));


faces = [1:circle_res;circle_res + (1:circle_res)];

patch_handle.Vertices = pts';
patch_handle.Faces = faces;
patch_handle.FaceColor = [1.0 1.0 1.0];
patch_handle.FaceAlpha = 0.2;


% axis equal;
% axis(repmat([-3 3], 1,3));
% view(3);
grid on;
end
