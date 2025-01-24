function hive = PosControl_ForceDistrib(hive,dt)

%% Centralized Trajectory Controller

    %% Desired Linear Accelerations
    
    k_p0 = 10*eye(3); %10
    k_p1 = 5*eye(3);  %5
    %k_i = 0*eye(3);
    
    %hive.pos_integral = hive.pos_integral + ...
    %    (hive.des.x_0_w(:,end) - hive.x_0_w)*dt;

    x_0r_w_ddot = hive.des.x_0_w_ddot ...
        + k_p0 * (hive.des.x_0_w - hive.x_0_w) ...
        + k_p1 * (hive.des.x_0_w_dot - hive.x_0_w_dot);
       % + k_i * hive.pos_integral;

    %% Desired Angular Accelerations
    k_w0 = 5*eye(3); %3
    k_w1 = 3*eye(3);                  %2
    %k_Ri = 0*eye(3);
    
    e_R = .5 * veemap((hive.R_0_w' * hive.des.R_0_w) - (hive.des.R_0_w' * hive.R_0_w));
    %hive.R_integral = hive.R_integral + e_R*dt;

    omega_0r_0_dot = hive.des.omega_0_0_dot ...
        + k_w0 * e_R ...
        + k_w1 * (hive.des.omega_0_0 - hive.omega_0_0);
        %+ k_Ri * hive.R_integral;

%% Force Distribution    

    %% Structure Configuration Computation (Matrix S)
    
    R_i_s= zeros(3,3*hive.n);
    P_i_R_i_s = zeros(3,3*hive.n);
    R_s_w = zeros(3*hive.n, 3*hive.n);
    R_i_w = zeros(3,3,3*hive.n);
    R_i_w_aux = zeros(3*hive.n, 3*hive.n);

    for q = 1:hive.n
        
        A = [0 0 0;0 1 0;0 0 1];
        R_i_s(:,3*(q-1)+1:(3*(q-1)+1)+2) = hive.quads(q).R_i0_0*A;
        P_i_R_i_s(:,3*(q-1)+1:(3*(q-1)+1)+2) = makeskew(hive.quads(q).r_i_0) * hive.quads(q).R_i0_0*A;
        R_s_w(3*(q-1)+1:(3*(q-1)+1)+2,3*(q-1)+1:(3*(q-1)+1)+2) = hive.R_0_w;
        
        R_i_w(:,:,q) = hive.R_0_w *hive.quads(q).R_i0_0 * hive.quads(q).R_i_0_x;
        R_i_w_aux(3*(q-1)+1:(3*(q-1)+1)+2,3*(q-1)+1:(3*(q-1)+1)+2) = hive.R_0_w *hive.quads(q).R_i0_0;
    end
    S = [R_i_s;P_i_R_i_s];
    
  %% Desired Wrench (Forces and Moments) Computation in the Structure Frame
  
    F_des_S = hive.R_0_w' * hive.M_0 * ([0,0,9.81]'+ x_0r_w_ddot);
    M_des_S = omega_0r_0_dot;
    wrench_des_S = [F_des_S;M_des_S];
  
  %% Desired Force Computation for Module i
    mode = 2;
    
    if mode==1
        % Quadratic Programming minimization
        
        Thrust_max = 1000;
        Thrust_min = 0;
        lb = zeros(3*hive.n,1);
        ub = zeros(3*hive.n,1);
        
        f = zeros(1,3*hive.n);
        H = 2*eye(3*hive.n, 3*hive.n);
        for q = 1:1:hive.n
            lb(3*(q-1)+1:3*(q-1)+3,1) = [0,-Thrust_max*sind(15),Thrust_min]';
            ub(3*(q-1)+1:3*(q-1)+3,1) = [0,Thrust_max*cosd(15),Thrust_max*cosd(15)]';
        end
        F_i = quadprog(H,f,[],[],S,wrench_des_S,lb,ub,x0);
        
    elseif mode==2
        % Unconstrained minimization (pseudo-inverse)
        
        F_i = pinv(S) * wrench_des_S;
        rank(S)
    end
    
%% Transformation from desired Force_i to Thrust_i and desired Alpha_i
    
    F_W_i = R_i_w_aux * F_i;
    zb = zeros(3,hive.n);
    xc = zeros(3, hive.n);
    yaw_cur_i = zeros(3, hive.n);
    
    for q = 1:hive.n
       hive.quads(q).thrust = F_W_i(3*(q-1)+1:(3*(q-1)+1)+2)' * R_i_w(:,3,q);
       if norm(F_W_i(3*(q-1)+1:(3*(q-1)+1)+2))==0
            zb(:,q) = zeros(3,1);
       else  
       zb(:,q) = F_W_i(3*(q-1)+1:(3*(q-1)+1)+2) / norm(F_W_i(3*(q-1)+1:(3*(q-1)+1)+2));
       yaw_cur_i(:,q) = rotm2eul(R_i_w(:,:,q), 'XYZ');
       xc(:,q) = [cos(yaw_cur_i(3,q)), sin(yaw_cur_i(3,q)),0]';
       Yb = cross(zb,xc);
       yb = Yb/norm(Yb);
       hive.quads(q).alpha = atan2(yb(3,q),zb(3,q));
       hive.quads(q).alpha = max(min(hive.quads(q).alpha, 0.26), -0.26);
       end
    end
 
end
