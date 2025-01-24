function [angle_min_alt2,f_min_alt2] = alt2_structOptm(dt,t_final,shape,number_mod,traj_type)

f_min_alt2 = inf;
angle_min_alt2 = zeros(1,number_mod);  
angle_2 = zeros(1,number_mod);

progress = waitbar(0, 'Modular Optmization...');



for q =1:1:number_mod
    waitbar(q/number_mod, progress);
for aux=1:1:2
            if aux==1
                angle_2(q) = pi/2;
            else
                angle_2(q) = 0;
            end
        
        modquad = init_modquad(shape, number_mod, angle_2);
        S = structureConfiguration(modquad);
        S_new = double(vpa(S));
        
        
        modquad.wrenchSum_des_S = zeros(6,1);
        modquad.pos_error = 0;
        modquad.R_0_w = eye(3);
        modquad.omega_0_0 = [0 0 0]';
        modquad.x_0_w = [0 0 0]';
        modquad.x_0_w_dot = [0 0 0]';
        
          for t = 0:dt:t_final
                modquad.t = t;
                modquad = trajectory(modquad,traj_type,dt);   
                modquad = PosControl_ForceDistrib(modquad,S_new);
                modquad = attControl_motorDistrib(modquad,dt);
                modquad = updateDeriv(modquad, dt);
          end

        pos_error = modquad.pos_error;
        f_vector = pinv(S_new) * modquad.wrenchSum_des_S * dt;
        modquad.f = norm(f_vector);
        f = modquad.f;
        error = (pos_error*dt);
        pos_error = 0;
        
        if error<f_min_alt2
            f_min_alt2 = error;
            angle_min_alt2(q) = angle_2(q);   
        end 
end
end
delete(progress);
end