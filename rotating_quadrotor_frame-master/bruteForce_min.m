function [angle_min_bf,f_min_bf] = bruteForce_min(dt,t_final,shape,number_mod,traj_type)


f_min_bf = inf;
angle_min_bf = zeros(1,number_mod);
numberConfig = 2^number_mod;
C = -(dec2bin(0:(numberConfig)-1) - 49);
C(C==1)= pi/2;
C = C';
angle_2 = zeros(1,number_mod);
progress = waitbar(0, 'Brute-Force Computation...');

for count=1:1:numberConfig
    waitbar(count/numberConfig, progress);
    for count2=1:1:number_mod
        
        angle_2(count2) = C(count2,count);
    end
    
        modquad = init_modquad(shape,number_mod,angle_2);
        r_i_0 = modquad.r_i_0;
        R_i0_0 = modquad.R_i0_0;
        S_new = structureConfiguration(r_i_0,R_i0_0,number_mod);
        rank(S_new);
        
        
        modquad.wrenchSum_des_S = zeros(6,1);
        modquad.pos_error = 0;
        modquad.R_0_w = eye(3);
        modquad.omega_0_0 = [0 0 0]';
        modquad.x_0_w = [0 0 0]';
        modquad.x_0_w_dot = [0 0 0]';
        modquad.x_0_w_ddot = [0 0 0]';
        
        for t = 0:dt:t_final
            modquad.t = t;
            modquad = trajectory(modquad,traj_type,dt);   
            [modquad,flag]  = PosControl_ForceDistrib(modquad,S_new);
            if flag ==1
              break
            end
            modquad = attControl_motorDistrib(modquad,dt);
            modquad = updateDeriv(modquad, dt);
        end
        if flag ==1
            continue
        end        
        pos_error = modquad.pos_error;
        f_vector = pinv(S_new) * modquad.wrenchSum_des_S * dt;
        modquad.f = norm(f_vector);
        f = modquad.f;
        error = (pos_error*dt);
        pos_error = 0;
        
        if rank(S_new)>=6
            
            if f<f_min_bf 
                f_min_bf = f;

                for count2=1:1:modquad.n
                    angle_min_bf(count2) = angle_2(count2);
                end

            end
        
        end
        
end
delete(progress);

end
