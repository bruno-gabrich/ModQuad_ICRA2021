function modquad = trajectory(modquad,traj_type,dt)

traj=traj_type;

if traj==1
    
    %% Linear Lissajous curve
    lin_a = [2 1 0.5]';
    lin_o = [1 2 1]';
    lin_d = [pi/2 0 0]';

    x0 = -lin_a .* sin(lin_d);

    modquad.des.x_0_w      = x0 + lin_a .* sin(lin_o * modquad.t + lin_d);
    modquad.des.x_0_w_dot  = lin_a .* lin_o .* cos(lin_o*modquad.t + lin_d);
    modquad.des.x_0_w_ddot = -lin_a .* lin_o.^2 .* sin(lin_o*modquad.t + lin_d);
    
elseif traj==2
    
    %% Spiral Trajectory
    modquad.des.x_0_w = [cos(modquad.t);sin(modquad.t);modquad.t];
    modquad.des.x_0_w_dot = [-sin(modquad.t);cos(modquad.t);1];
    modquad.des.x_0_w_ddot = [cos(modquad.t);-sin(modquad.t);0];
    
elseif traj==3
    
    %% Folium of Descaters
    modquad.des.x_0_w =  [3*modquad.t/(1+(modquad.t)^3);3*((modquad.t)^2)/(1+(modquad.t)^3);1];
    modquad.des.x_0_w_dot = [(3-6*(modquad.t)^3)/(1+(modquad.t)^2)^3;-3*modquad.t*((modquad.t)^3-2)/(1+(modquad.t)^2)^3;0];
    modquad.des.x_0_w_ddot = [18*(modquad.t^2)*((modquad.t^3)-2)/(1+(modquad.t^3))^3;6*((modquad.t^6)-7*(modquad.t^3)+1)/(1+(modquad.t^3))^3;0];
end


%% Angular
mode = 3;

if mode == 1
    % Sinusoidal rotations
    max_w = [.2 .2 .2]';
    ome_w = [.5 .5 .5]';
    phi_w = [0 pi/3 2*pi/3]';
    
    modquad.des.omega_0_0 = ...
        max_w .* sin(ome_w.*modquad.t + phi_w);
    modquad.des.omega_0_0_dot = ...
        max_w .* ome_w .* cos(ome_w.*modquad.t + phi_w);
    modquad.des.omega_0_0_ddot = ...
        -max_w .* ome_w.^2 .* sin(ome_w.*modquad.t + phi_w);
elseif mode == 2
    % Constant angular velocity
    modquad.des.omega_0_0 = [0.0 0 1.0]';
    modquad.des.omega_0_0_dot = [0 0 0]';
    modquad.des.omega_0_0_ddot = [0 0 0]';
    modquad.des.R_0_w = expm(makeskew(modquad.des.omega_0_0*dt)) * modquad.des.R_0_w;

elseif mode == 3
    % Sinusoidal rotations
    max_w = [1 1 1]';
    ome_w = [.5 .5 .5]';
    phi_w = [0 pi/3 2*pi/3]';
    
    modquad.des.omega_0_0 = ...
        max_w .* sin(ome_w.*modquad.t + phi_w);
    modquad.des.omega_0_0_dot = ...
        max_w .* ome_w .* cos(ome_w.*modquad.t + phi_w);

else
    % Constant attitude
    if modquad.t<=pi/4
        angle = modquad.t;
    
    elseif modquad.t >= pi/4 && modquad.t<=pi/2
        angle = pi/4-(modquad.t -pi/4);
   
        
    elseif modquad.t >=pi/2 && modquad.t<=2*pi/3
        angle = modquad.t - pi/4;
    
    elseif modquad.t>= 2*pi/3
        angle = pi/4 - (modquad.t - 2*pi/3);
    end
    modquad.des.R_0_w = rotx(0)*roty(0)*rotz(0);
    modquad.des.omega_0_0 = [0 0 0]';
    modquad.des.omega_0_0_dot = zeros(3,1);
    %modquad.des.omega_0_0_ddot = zeros(3,1);
    
    %added by bruno
    %modquad.des.omega_0_0_3dot = zeros(3,1);
    
end

end
