function modquad = plot_forces(modquad, i)
base_pts = zeros(3,5*modquad.n);
tip_pts = zeros(3,5*modquad.n);
arm_length = 0.15;
if nargin == 1
a=1;
max_a = modquad.n*5;
q = 1;

    while a<max_a
        
        base_pts(:,a) = modquad.quads(q).x_i_w;
        tip_pts(:,a) = modquad.quads(q).R_i_w(:,3) * modquad.quads(q).thrust;
        a = a + 1;
        base_pts(:,a) = modquad.quads(q).x_i_w + modquad.R_i_w * [arm_length * cosd(45); -arm_length *sind(45); 0.1];
        tip_pts(:,a) = modquad.quads(q).R_i_w(:,3) * modquad.quads(q).f1;
        a= a + 1;
        base_pts(:,a) = modquad.quads(q).x_i_w + modquad.R_i_W * [-arm_length * cosd(45); -arm_length *sind(45); 0.1]; 
        tip_pts(:,a) = modquad.quads(q).R_i_w(:,3) * modquad.quads(q).f2; 
        a = a + 1;
        base_pts(:,a) = modquad.quads(q).x_i_w + modquad.R_i_W * [-arm_length * cosd(45); arm_length *sind(45); 0.1]; 
        tip_pts(:,a) = modquad.quads(q).R_i_w(:,3) * modquad.quads(q).f3; 
        a = a + 1;
        base_pts(:,q) = modquad.quads(q).x_i_w + modquad.R_i_W * [arm_length * cosd(45); arm_length *sind(45); 0.1];
        tip_pts(:,q) = modquad.quads(q).R_i_w(:,3) * modquad.quads(q).f4; 
        a = a + 1;
        q = q + 1;
    end
    
    modquad.forces_handle.XData = base_pts(1,:);
    modquad.forces_handle.YData = base_pts(2,:);
    modquad.forces_handle.ZData = base_pts(3,:);
    modquad.forces_handle.UData = tip_pts(1,:);
    modquad.forces_handle.VData = tip_pts(2,:);
    modquad.forces_handle.WData = tip_pts(3,:);

else
    %% plot a historical one
%    for q = 1:modquad.n
%        base_pts(:,q) = modquad.history.quads(q).x_i_w(:,i);
%        tip_pts(:,q)  = modquad.history.quads(q).R_i_w(:,3,i) * modquad.history.quads(q).thrust;
%    end
    
%    modquad.forces_handle.XData = base_pts(1,:);
%    modquad.forces_handle.YData = base_pts(2,:);
%    modquad.forces_handle.ZData = base_pts(3,:);
%    modquad.forces_handle.UData = tip_pts(1,:);
%    modquad.forces_handle.VData = tip_pts(2,:);
%    modquad.forces_handle.WData = tip_pts(3,:);
    
max_a = modquad.n*5;
q = 1;
a=1;

    while a<max_a
        
        base_pts(:,a) = modquad.history.quads(q).x_i_w(:,i);
        tip_pts(:,a) = modquad.history.quads(q).R_i_w(:,3,i) * modquad.history.quads(q).thrust;
        a = a + 1;
        base_pts(:,a) = modquad.history.quads(q).x_i_w(:,i) + modquad.history.quads(q).R_i_w(:,:,i) * [arm_length * cosd(45); -arm_length *sind(45); 0.1];
        tip_pts(:,a) = modquad.history.quads(q).R_i_w(:,3,i) * modquad.history.quads(q).f1;
        a= a + 1;
        base_pts(:,a) = modquad.history.quads(q).x_i_w(:,i) + modquad.history.quads(q).R_i_w(:,:,i) * [-arm_length * cosd(45); -arm_length *sind(45); 0.1]; 
        tip_pts(:,a) = modquad.history.quads(q).R_i_w(:,3,i) * modquad.history.quads(q).f2; 
        a = a + 1;
        base_pts(:,a) = modquad.history.quads(q).x_i_w(:,i) + modquad.history.quads(q).R_i_w(:,:,i) * [-arm_length * cosd(45); arm_length *sind(45); 0.1];
        tip_pts(:,a) = modquad.history.quads(q).R_i_w(:,3,i) * modquad.history.quads(q).f3; 
        a = a + 1;
        base_pts(:,a) = modquad.history.quads(q).x_i_w(:,i) + modquad.history.quads(q).R_i_w(:,:,i) * [arm_length * cosd(45); arm_length *sind(45); 0.1];
        tip_pts(:,a) = modquad.history.quads(q).R_i_w(:,3,i) * modquad.history.quads(q).f4; 
        a = a + 1;
        q = q + 1;
    end
    
    modquad.forces_handle.XData = base_pts(1,:);
    modquad.forces_handle.YData = base_pts(2,:);
    modquad.forces_handle.ZData = base_pts(3,:);
    modquad.forces_handle.UData = tip_pts(1,:)*10;
    modquad.forces_handle.VData = tip_pts(2,:)*10;
    modquad.forces_handle.WData = tip_pts(3,:)*10;
end

end
