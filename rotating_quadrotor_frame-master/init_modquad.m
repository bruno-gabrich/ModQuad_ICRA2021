function modquad = init_modquad(type, arg,angle) 
    %% General Setup
    modquad = struct();
    modquad.box_size = [1; 1; 0.2];
    modquad.init_plot = 1;

    % Mass properties
    modquad.I_i = [1.395e-5 0 0;
                 0      1.395e-5 0;
                 0         0      2.173e-5];
    modquad.I_0 = [1 0 0;
                0 1 0;
                0 0 1];

    % modquad state
    modquad.x_0_w = zeros(3,1);
    modquad.x_0_w_dot = zeros(3,1);
    modquad.R_0_w = eye(3);
    modquad.omega_0_0 = zeros(3,1);
    modquad.omega_0_0_dot = zeros(3,1);

    % modquad desired state
    modquad.des.x_0_w = zeros(3,1);
    modquad.des.x_0_w_dot = zeros(3,1);
    modquad.des.x_0_w_ddot = zeros(3,1);
    modquad.des.R_0_w = eye(3);
    
    % added by bruno
    

    % modquad state history
    modquad.history.t = 0;
    modquad.history.x_0_w = zeros(3,1);
    modquad.history.des.x_0_w = zeros(3,1);
    modquad.history.R_0_w = eye(3);
    modquad.history.eul_0_w = [0 0 0]';
    modquad.history.des.R_0_w = eye(3);
    modquad.history.des.eul_0_w = [0 0 0]';
    modquad.max = inf;

    % Integral controls
    modquad.pos_integral = zeros(3,1);
    modquad.R_integral = zeros(3,1);

    %% Create structure configuration
    
    
    if strcmp(type, 'square1')
       
        % Circular Struct
        
    
            radius = 1/(2*tan(pi/arg)) + 0.5;
            for q = 1:arg
                if q==1
                    theta = angle(q);
                    r = [cosd(45);sind(45);0];
                elseif q==2
                    theta = angle(q);
                    r = [cosd(-135);sind(-135);0];
                elseif q==3
                    theta = angle(q);
                    r = [cosd(-45);sind(-45);0];
                elseif q==4
                    theta = angle(q);
                    r = [cosd(135);sind(135);0];
                end
            modquad.quads(q).r_i_0 = radius * r *0.428;
            modquad.quads(q).R_i0_0 = rotz(theta); 
            end
        
        
    elseif strcmp(type, 'line2')
        % Circular Struct
        radius = 1/(2*tan(pi/arg)) + 0.5;
        for q = 1:arg
            if q==1
                theta = 0;
                r = [-1;0;0];
            elseif q==2
                theta = 0;
                r = [1;0;0];
            end
            modquad.quads(q).r_i_0 = radius*0.428*r;
            modquad.quads(q).R_i0_0 = rotz(theta);
        end
        
        
    elseif strcmp(type, 'eucledian')
        % Circular Struct
 
            for q = 1:arg
                if (q>=1) && (q<=4)
                    theta = angle(q);
                    r = [cosd(45 + 90*(q-1));sind(45 + 90*(q-1));0];
                    radius = sqrt(0.3^2+0.3^2);
                elseif (q>4) && (q<=16)
                    theta = angle(q);
                    if q>4 && q<=7
                        pos = [0.9 0.3;0.9 0.9;0.3 0.9];
                        r = [pos(q-4,:),0]';
                        radius = 1;
                    end
                    if q>7 && q<=10
                        pos = [-0.9 0.3;-0.9 0.9;-0.3 0.9];
                        r = [pos(q-7,:),0]';
                        radius = 1;
                    end
                    if q>10 && q<=13
                        pos = [-0.9 -0.3;-0.9 -0.9;-0.3 -0.9];
                        r = [pos(q-10,:),0]';
                        radius = 1;
                    end 
                     if q>13 && q<=16
                        pos = [0.9 -0.3;0.9 -0.9;0.3 -0.9];
                        r = [pos(q-13,:),0]';
                        radius = 1;
                     end           
                elseif (q>16) && (q<=36)
                    theta = angle(q);
                    if q>16 && q<=21
                        pos = [1.5 0.3;1.5 0.9;1.5 1.5;0.9 1.5;0.3 1.5];
                        r = [pos(q-16,:),0]';
                        radius = 1;
                    end
                    if q>21 && q<=26
                        pos = [-1.5 0.3;-1.5 0.9;-1.5 1.5;-0.9 1.5;-0.3 1.5];
                        r = [pos(q-21,:),0]';
                        radius = 1;
                    end
                    if q>26 && q<=31
                        pos = [-1.5 -0.3;-1.5 -0.9;-1.5 -1.5;-0.9 -1.5;-0.3 -1.5];
                        r = [pos(q-26,:),0]';
                        radius = 1;
                    end 
                     if q>31 && q<=36
                        pos = [1.5 -0.3;1.5 -0.9;1.5 -1.5;0.9 -1.5;0.3 -1.5];
                        r = [pos(q-31,:),0]';
                        radius = 1;
                     end 
                  elseif (q>36) && (q<=64)
                    theta = angle(q);
                    if q>36 && q<=43
                        pos = [2.1 0.3;2.1 0.9;2.1 1.5;2.1 2.1;1.5 2.1;0.9 2.1;0.3 2.1];
                        r = [pos(q-36,:),0]';
                        radius = 1;
                    end
                    if q>43 && q<=50
                        pos = [-2.1 0.3;-2.1 0.9;-2.1 1.5;-2.1 2.1;-1.5 2.1;-0.9 2.1;-0.3 2.1];
                        r = [pos(q-43,:),0]';
                        radius = 1;
                    end
                    if q>50 && q<=57
                        pos = [-2.1 -0.3;-2.1 -0.9;-2.1 -1.5;-2.1 -2.1;-1.5 -2.1;-0.9 -2.1;-0.3 -2.1];
                        r = [pos(q-50,:),0]';
                        radius = 1;
                    end 
                     if q>57 && q<=64
                        pos = [2.1 -0.3;2.1 -0.9;2.1 -1.5;2.1 -2.1;1.5 -2.1;0.9 -2.1;0.3 -2.1];
                        r = [pos(q-57,:),0]';
                        radius = 1;
                     end 
                  elseif (q>64) && (q<=100)
                    theta = angle(q);
                    if q>64 && q<=73
                        pos = [2.7 0.3;2.7 0.9;2.7 1.5;2.7 2.1;2.7 2.7;2.1 2.7;1.5 2.7;0.9 2.7;0.3 2.7];
                        r = [pos(q-64,:),0]';
                        radius = 1;
                    end
                    if q>73 && q<=82
                        pos = [-2.7 0.3;-2.7 0.9;-2.7 1.5;-2.7 2.1;-2.7 2.7;-2.1 2.7;-1.5 2.7;-0.9 2.7;-0.3 2.7];
                        r = [pos(q-73,:),0]';
                        radius = 1;
                    end
                    if q>82 && q<=91
                        pos = [-2.7 -0.3;-2.7 -0.9;-2.7 -1.5;-2.7 -2.1;-2.7 -2.7;-2.1 -2.7;-1.5 -2.7;-0.9 -2.7;-0.3 -2.7];
                        r = [pos(q-82,:),0]';
                        radius = 1;
                    end 
                     if q>91 && q<=100
                        pos = [2.7 -0.3;2.7 -0.9;2.7 -1.5;2.7 -2.1;2.7 -2.7;2.1 -2.7;1.5 -2.7;0.9 -2.7;0.3 -2.7];
                        r = [pos(q-91,:),0]';
                        radius = 1;
                     end 
                 elseif (q>100) && (q<=144)
                    theta = angle(q);
                    if q>100 && q<=111
                        pos = [3.3 0.3;3.3 0.9;3.3 1.5;3.3 2.1;3.3 2.7;3.3 3.3;2.7 3.3;2.1 3.3;1.5 3.3;0.9 3.3;0.3 3.3];
                        r = [pos(q-100,:),0]';
                        radius = 1;
                    end
                    if q>111 && q<=122
                        pos = [-3.3 0.3;-3.3 0.9;-3.3 1.5;-3.3 2.1;-3.3 2.7;-3.3 3.3;-2.7 3.3;-2.1 3.3;-1.5 3.3;-0.9 3.3;-0.3 3.3];
                        r = [pos(q-111,:),0]';
                        radius = 1;
                    end
                    if q>122 && q<=133
                        pos = -[3.3 0.3;3.3 0.9;3.3 1.5;3.3 2.1;3.3 2.7;3.3 3.3;2.7 3.3;2.1 3.3;1.5 3.3;0.9 3.3;0.3 3.3];
                        r = [pos(q-122,:),0]';
                        radius = 1;
                    end 
                     if q>133 && q<=144
                        pos = -[-3.3 0.3;-3.3 0.9;-3.3 1.5;-3.3 2.1;-3.3 2.7;-3.3 3.3;-2.7 3.3;-2.1 3.3;-1.5 3.3;-0.9 3.3;-0.3 3.3];
                        r = [pos(q-133,:),0]';
                        radius = 1;
                     end 
                elseif (q>144) && (q<=196)
                    theta = angle(q);
                    if q>144 && q<=157
                        pos = [3.9 0.3;3.9 0.9;3.9 1.5;3.9 2.1;3.9 2.7;3.9 3.3;3.9 3.9;3.3 3.9;2.7 3.9;2.1 3.9;1.5 3.9;0.9 3.9;0.3 3.9];
                        r = [pos(q-144,:),0]';
                        radius = 1;
                    end
                    if q>157 && q<=170
                        pos = [-3.9 0.3;-3.9 0.9;-3.9 1.5;-3.9 2.1;-3.9 2.7;-3.9 3.3;-3.9 3.9;-3.3 3.9;-2.7 3.9;-2.1 3.9;-1.5 3.9;-0.9 3.9;-0.3 3.9];
                        r = [pos(q-157,:),0]';
                        radius = 1;
                    end
                    if q>170 && q<=183
                        pos = -[3.9 0.3;3.9 0.9;3.9 1.5;3.9 2.1;3.9 2.7;3.9 3.3;3.9 3.9;3.3 3.9;2.7 3.9;2.1 3.9;1.5 3.9;0.9 3.9;0.3 3.9];
                        r = [pos(q-170,:),0]';
                        radius = 1;
                    end 
                     if q>183 && q<=196
                        pos = -[-3.9 0.3;-3.9 0.9;-3.9 1.5;-3.9 2.1;-3.9 2.7;-3.9 3.3;-3.9 3.9;-3.3 3.9;-2.7 3.9;-2.1 3.9;-1.5 3.9;-0.9 3.9;-0.3 3.9];
                        r = [pos(q-183,:),0]';
                        radius = 1;
                     end 
                 elseif (q>196) && (q<=256)
                    theta = angle(q);
                    if q>196 && q<=211
                        pos = [4.5 0.3;4.5 0.9;4.5 1.5;4.5 2.1;4.5 2.7;4.5 3.3;4.5 3.9;4.5 4.5;3.9 4.5;3.3 4.5;2.7 4.5;2.1 4.5;1.5 4.5;0.9 4.5;0.3 4.5];
                        r = [pos(q-196,:),0]';
                        radius = 1;
                    end
                    if q>211 && q<=226
                        pos = [-4.5 0.3;-4.5 0.9;-4.5 1.5;-4.5 2.1;-4.5 2.7;-4.5 3.3;-4.5 3.9;-4.5 4.5;-3.9 4.5;-3.3 4.5;-2.7 4.5;-2.1 4.5;-1.5 4.5;-0.9 4.5;-0.3 4.5];
                        r = [pos(q-211,:),0]';
                        radius = 1;
                    end
                    if q>226 && q<=241
                        pos = -[4.5 0.3;4.5 0.9;4.5 1.5;4.5 2.1;4.5 2.7;4.5 3.3;4.5 3.9;4.5 4.5;3.9 4.5;3.3 4.5;2.7 4.5;2.1 4.5;1.5 4.5;0.9 4.5;0.3 4.5];
                        r = [pos(q-226,:),0]';
                        radius = 1;
                    end 
                     if q>241 && q<=256
                        pos = -[-4.5 0.3;-4.5 0.9;-4.5 1.5;-4.5 2.1;-4.5 2.7;-4.5 3.3;-4.5 3.9;-4.5 4.5;-3.9 4.5;-3.3 4.5;-2.7 4.5;-2.1 4.5;-1.5 4.5;-0.9 4.5;-0.3 4.5];
                        r = [pos(q-241,:),0]';
                        radius = 1;
                     end
                  
                end
                modquad.quads(q).r_i_0 = radius * r ;
                modquad.quads(q).R_i0_0 = rotz(theta);
            end     
        


    elseif strcmp(type, 'single')
        % Single Robot
        modquad.quads(1).r_i_0 = zeros(3,1);
        modquad.quads(1).R_i0_0 = eye(3);
        modquad.quads(2).r_i_0 = [1 0 0]';
        modquad.quads(2).R_i0_0 = rotz(pi/2);

    elseif strcmp(type, 'line')
        % Line struct
        for q = 1:arg
            theta = pi/2 * (q);
            modquad.quads(q).r_i_0 = (q - ((arg-1)/2) -1) * [0; 1; 0];
            modquad.quads(q).R_i0_0 = rotz(theta);
        end

    elseif strcmp(type, 'vi')
        %% VI struct
        modquad.quads(1).r_i_0 = [1;  0;  0];
        modquad.quads(2).r_i_0 = [-1; 0;  0];
        modquad.quads(3).r_i_0 = [0;  1;  0];
        modquad.quads(4).r_i_0 = [0; -1;  0];
        modquad.quads(5).r_i_0 = [0;  0;  1];
        modquad.quads(6).r_i_0 = [0;  0; -1];

        modquad.quads(1).R_i0_0 = rotz(0);
        modquad.quads(2).R_i0_0 = rotz(0);
        modquad.quads(3).R_i0_0 = rotz(pi/2);
        modquad.quads(4).R_i0_0 = rotz(pi/2);
        modquad.quads(5).R_i0_0 = roty(pi/2);
        modquad.quads(6).R_i0_0 = roty(pi/2);
        
        modquad.box_size = [1; 1; 1];
    end

%% Setup indivudual quads
modquad.n = length(modquad.quads);
modquad.M_0 = 0.33;


for q = 1:modquad.n
    modquad.quads(q).I_i = 0.1*eye(3);
    modquad.quads(q).alpha = 0;
    modquad.quads(q).alpha_old = 0;
    modquad.quads(q).alpha_dot_old = 0;
    modquad.quads(q).alpha_dot_raw =0;
    modquad.quads(q).alpha_dot_raw_old =0;
    modquad.quads(q).alpha_dot = 0;  
    modquad.quads(q).R_i_0 = modquad.quads(q).R_i0_0 * rotx(modquad.quads(q).alpha);
    modquad.quads(q).thrust = .1;
    modquad.quads(q).R_i_0_x = rotx(0);
    modquad.quads(q).M_i_x = 0;
    modquad.quads(q).alpha_acc = zeros(3,1);
    modquad.quads(q).alpha_vel = zeros(3,1);
    
    modquad.quads(q).tau_x = 0;
    modquad.quads(q).tau_y = 0;
    modquad.quads(q).tau_z = 0;

    % Quad state history
    modquad.history.quads(q).x_i_w = zeros(3,1);
    modquad.history.quads(q).R_i_w = eye(3);
    modquad.history.quads(q).thrust = 0;
    
    % quad motor forces
    modquad.quads(q).f1 = 0;
    modquad.quads(q).f2 = 0;
    modquad.quads(q).f3 = 0;
    modquad.quads(q).f4 = 0;
    modquad.history.quads(q).f1 = 0;
    modquad.history.quads(q).f2 = 0;
    modquad.history.quads(q).f3 = 0;
    modquad.history.quads(q).f4 = 0;
    
    modquad.r_i_0(:,q) = modquad.quads(q).r_i_0;
    modquad.R_i0_0(:,:,q) = modquad.quads(q).R_i0_0; 
    
end

%modquad = sym_solve(modquad);

end
