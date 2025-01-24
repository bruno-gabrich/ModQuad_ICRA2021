function modquad = updateDeriv(modquad,dt)
    forces_sum = 0;
    moments_sum = 0;
    arm_length = 0.15;
    y=0;
    x=0;
    z=0;
    for q = 1:modquad.n
        
        %% Computing quads position given structure pose
        
        modquad.quads(q).x_i_w = modquad.x_0_w + modquad.R_0_w * modquad.quads(q).r_i_0;
        
        %% Deriving Rotation around x-axis for each quadrotor given propeller forces
        
        modquad.quads(q).M_i_x = 0;
        modquad.quads(q).alpha_vel = zeros(3,1);
        modquad.quads(q).alpha_acc = zeros(3,1);
        
        
        modquad.quads(q).M_i_x = arm_length * cosd(45) * (modquad.quads(q).f4 + modquad.quads(q).f3 - modquad.quads(q).f2 - modquad.quads(q).f1);
        modquad.quads(q).alpha_acc = [modquad.quads(q).M_i_x/modquad.I_i(1,1);0;0];
        
        modquad.quads(q).alpha_vel = modquad.quads(q).alpha_vel + modquad.quads(q).alpha_acc * dt;
        modquad.quads(q).R_i_0_x   = expm(makeskew(modquad.quads(q).alpha_vel*dt)) * modquad.quads(q).R_i_0_x;  
 
        %% Computing Resultant Structure Thrust
        
        modquad.quads(q).R_i_0 = modquad.quads(q).R_i0_0 * modquad.quads(q).R_i_0_x;
        modquad.quads(q).R_i_w = modquad.R_0_w * modquad.quads(q).R_i_0;
       
        thrust = modquad.quads(q).f1 + modquad.quads(q).f2 + modquad.quads(q).f3 + modquad.quads(q).f4;
        forces_sum = forces_sum + modquad.quads(q).R_i_w(:,3)*(thrust);


        %% Computing Resultant Structure Moments
        
        R_iz_0 = modquad.quads(q).R_i_0*[0,0,1]';
        
        moments_1 = cross(modquad.quads(q).r_i_0 + modquad.quads(q).R_i_0 * [arm_length,0,0]', R_iz_0 * modquad.quads(q).f1);
        moments_2 = cross(modquad.quads(q).r_i_0 + modquad.quads(q).R_i_0 * [-arm_length,0,0]', R_iz_0 * modquad.quads(q).f2);
        moments_3 = cross(modquad.quads(q).r_i_0 + modquad.quads(q).R_i_0 * [0,arm_length,0]', R_iz_0 * modquad.quads(q).f3);
        moments_4 = cross(modquad.quads(q).r_i_0 + modquad.quads(q).R_i_0 * [0,-arm_length,0]', R_iz_0 * modquad.quads(q).f4);
           
        moments_sum = moments_sum + (moments_1 + moments_2 +moments_3 + moments_4);
        
        y = y + (modquad.quads(q).r_i_0(2))^2;
        x = x + (modquad.quads(q).r_i_0(1))^2;
        z_aux = y+x;
        z = z+z_aux;
        
      

    end
    I_S = modquad.n * modquad.I_0 + modquad.M_0 *[y 0 0;0 x 0;0 0 z];
    %% Push derivatives back
    
    modquad.x_0_w_ddot = forces_sum / (modquad.M_0*modquad.n) + [0 0 -9.81]';
    modquad.x_0_w_dot  = modquad.x_0_w_dot + modquad.x_0_w_ddot * dt;
    modquad.x_0_w      = modquad.x_0_w + modquad.x_0_w_dot * dt;
    

    modquad.omega_0_0_dot  = inv(I_S) * (moments_sum - cross(modquad.omega_0_0, inv(I_S)*modquad.omega_0_0)) ;
    modquad.omega_0_0      = modquad.omega_0_0 + modquad.omega_0_0_dot * dt;
    modquad.R_0_w          = expm(makeskew(modquad.omega_0_0*dt)) * modquad.R_0_w;
   
    
end
