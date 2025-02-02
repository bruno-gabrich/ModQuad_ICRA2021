function [modquad,flag] = PosControl_ForceDistrib(modquad,S)
flag=0;
%% Centralized Trajectory Controller

    %% Desired Linear Accelerations
    
    k_p0 = [10 0 0;0 10 0;0 0 10]; %10
    k_p1 = [5 0 0;0 5 0;0 0 5];   %5
    %k_i = 0*eye(3);
    
    %modquad.pos_integral = modquad.pos_integral + ...
    %    (modquad.des.x_0_w(:,end) - modquad.x_0_w)*dt;

    x_0r_w_ddot = (modquad.des.x_0_w_ddot) ...
        + k_p0* (modquad.des.x_0_w - modquad.x_0_w) ...
        + k_p1 * (modquad.des.x_0_w_dot - modquad.x_0_w_dot);
       % + k_i * modquad.pos_integral;

    %% Desired Angular Accelerations
    k_w0 = 5*eye(3); %3
    k_w1 = 3*eye(3);                  %2
    %k_Ri = 0*eye(3);
    
    e_R = .5 * veemap((modquad.R_0_w' * modquad.des.R_0_w) - (modquad.des.R_0_w' * modquad.R_0_w));
    %modquad.R_integral = modquad.R_integral + e_R*dt;

%    omega_0r_0_dot = modquad.des.omega_0_0_dot ...
%        + k_w0 * e_R ...
%        + k_w1 * (modquad.des.omega_0_0 - modquad.omega_0_0);
    
   omega_0r_0_dot =  k_w0 * e_R ...
        - k_w1 * modquad.omega_0_0;
 
        %+ k_Ri * modquad.R_integral;

%% Force Distribution    

    %% Structure Configuration Computation (Matrix S)
    

    R_s_w =  zeros(3*modquad.n, 3*modquad.n);
    R_i_w = zeros(3,3,3*modquad.n);
    R_i_w_aux = zeros(3*modquad.n,3*modquad.n);
    y=0;
    x = 0;
    z = 0;
    for q = 1:modquad.n

        R_s_w(3*(q-1)+1:(3*(q-1)+1)+2,3*(q-1)+1:(3*(q-1)+1)+2) = modquad.R_0_w;
        R_i_w(:,:,q) = modquad.R_0_w *modquad.quads(q).R_i0_0 * modquad.quads(q).R_i_0_x;
        R_i_w_aux(3*(q-1)+1:(3*(q-1)+1)+2,3*(q-1)+1:(3*(q-1)+1)+2) = modquad.R_0_w * modquad.quads(q).R_i0_0;
        
        y = y + (modquad.quads(q).r_i_0(2))^2;
        x = x + (modquad.quads(q).r_i_0(1))^2;
        z_aux = y+x;
        z = z+z_aux;
        
    end   
        I_S = modquad.n * modquad.I_0 + modquad.M_0 *[y 0 0;0 x 0;0 0 z];
    
  %% Desired Wrench (Forces and Moments) Computation in the Structure Frame
  
    F_des_S = modquad.n* modquad.R_0_w' * modquad.M_0 * (x_0r_w_ddot) + modquad.n * modquad.R_0_w'* modquad.M_0 * [0,0,9.81]';
    M_des_S = cross(modquad.omega_0_0, I_S * modquad.omega_0_0) +  I_S*omega_0r_0_dot;
    W_des_S = [F_des_S;M_des_S];
        
  %% Desired Wrench Sum
     modquad.wrenchSum_des_S = modquad.wrenchSum_des_S + W_des_S;
     e = modquad.des.x_0_w - modquad.x_0_w;
     e_total = norm([e;e_R]);  
     modquad.pos_error = modquad.pos_error + e_total;
  %% Desired Force Computation for Module i
    mode = 2;
    
    if mode==1
        % Quadratic Programming minimization
        
        Thrust_max = 1;
        %Thrust_min = 0;
        lb = zeros(3*modquad.n,1);
        ub = zeros(3*modquad.n,1);
        x0 = zeros(3*modquad.n,1);
        rollMaxdeg = 45;
        F_z_max = sqrt(((Thrust_max*tand(rollMaxdeg))^2)/(1+(tand(rollMaxdeg)^2)));
        F_y_max = F_z_max/tand(rollMaxdeg);
        f = zeros(1,3*modquad.n);
        H = 0.5*eye(3*modquad.n, 3*modquad.n);
        for q = 1:1:modquad.n
            lb(3*(q-1)+1:3*(q-1)+3,1) = [0,-30*sin(rollMaxdeg),0]';
            ub(3*(q-1)+1:3*(q-1)+3,1) = [0,30*sin(rollMaxdeg),30*cos(rollMaxdeg)]';
        end
        %options = optimset('quadprog');
        %options = optimset('Display','off');
        try
            [F_Mi] = quadprog(H,f,[],[],S,W_des_S,lb,ub);
        catch
        
        
        end
    elseif mode==2
        number_mod = modquad.n;
        Thrust_max = 10;
        x0 = zeros(3*modquad.n,1);
        nonlcon = @(x) con_fun(x,number_mod,Thrust_max);
        try
            [F_Mi] = fmincon(@(x) obj_fun(x,number_mod),x0,[],[],S,W_des_S,[],[],nonlcon);
        catch
            
        end
    elseif mode==3
        % Unconstrained minimization (pseudo-inverse)
        F_Mi = pinv(S)* W_des_S;
    end
    
    tf = isempty(F_Mi);
    if tf == 1
        flag=1;
        return
    end
    
%% Transformation from desired Force_i to Thrust_i and desired Alpha_i

    F_W_i = R_i_w_aux * F_Mi;
    zb = zeros(3,modquad.n);
    xc = zeros(3, modquad.n);
    yaw_cur_i = zeros(3, modquad.n);
    
    for q = 1:modquad.n
       modquad.quads(q).thrust = dot(F_W_i(3*(q-1)+1:(3*(q-1)+1)+2),R_i_w(:,3,q));
       if norm(F_W_i(3*(q-1)+1:(3*(q-1)+1)+2))==0
            zb(:,q) = zeros(3,1);
       else  
           zb(:,q) = F_W_i(3*(q-1)+1:(3*(q-1)+1)+2) / norm(F_W_i(3*(q-1)+1:(3*(q-1)+1)+2));
           yaw_cur_i(:,q) = rotm2eul(R_i_w(:,:,q), 'XYZ');
           xc(:,q) = [cos(yaw_cur_i(3,q)), sin(yaw_cur_i(3,q)),0]';



           Yb = cross(zb,xc);
           yb = Yb/norm(Yb);
           atan2(yb(3,q),zb(3,q));
           modquad.quads(q).alpha = atan2(yb(3,q),zb(3,q));
           %modquad.quads(q).alpha = max(min(modquad.quads(q).alpha, pi/4), -pi/4);
           var = modquad.quads(q).alpha;
           if var>=0.78 || var<=-0.78
                var
            end

        if modquad.quads(q).thrust<modquad.max
           modquad.max = modquad.quads(q).thrust;
        end
       
           maxiii = modquad.max;
           maxiii;
        end
    end
 
end
