%% Function to plot a cube mesh at a given location and size
function patch_handle = plot_quad(center, rot, patch_handle)
center = reshape(center, 3,1);

arm_length = .15;
box_length = .5; % half
prop_radius = 0.09;
circle_res = 20;

theta = 0:(2*pi/(circle_res-1)):2*pi;

base_circle = [cos(theta); sin(theta); zeros(1, circle_res)];
prop1_pts = prop_radius * base_circle + [arm_length * cosd(45); -arm_length *sind(45); 0.02];
prop2_pts = prop_radius * base_circle + [-arm_length * cosd(45); -arm_length *sind(45); 0.02];
prop3_pts = prop_radius * base_circle + [-arm_length * cosd(45); arm_length *sind(45); 0.02];
prop4_pts = prop_radius * base_circle + [arm_length * cosd(45); arm_length *sind(45); 0.02];
arm1_pts = [arm_length *cosd(45), -arm_length*cosd(45); -arm_length*cosd(45), arm_length*cosd(45); 0.02, 0.02];
arm2_pts = [arm_length *cosd(45), -arm_length*cosd(45); arm_length *cosd(45), -arm_length*cosd(45); 0.02, 0.02];
%arm2_pts = [0, 0;  0, 0; 0, 0];
%arm1_pts = [0, 0;  0, 0; 0, 0];
pts = [prop1_pts, prop2_pts, prop3_pts, prop4_pts, arm1_pts, arm2_pts];  
pts = rot*(pts) + repmat(center,1,size(pts,2));


faces = [1:circle_res;...
    circle_res + (1:circle_res);
    2*circle_res + (1:circle_res);
    3*circle_res + (1:circle_res);
    4*circle_res + (1:2), NaN(1,circle_res - 2);
    4*circle_res + (3:4), NaN(1,circle_res - 2);
];

patch_handle.Vertices = pts';
patch_handle.Faces = faces;
patch_handle.FaceColor = [0 0 0];
patch_handle.FaceAlpha = 0.5;


% axis equal;
% axis(repmat([-3 3], 1,3));
% view(3);
% grid on;
end