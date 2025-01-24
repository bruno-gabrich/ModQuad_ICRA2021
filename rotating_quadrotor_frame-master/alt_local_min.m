function [angle_min_alt,f_min_alt] = alt_local_min(dt,t_final,shape,number_mod,traj_type,modquad)

f_min_alt = inf;
angle_min_alt = zeros(1,number_mod);  
dist_to_center = zeros(1,number_mod);
angle_2 = ones(1,number_mod);
angle_2 = (pi/2)*angle_2;
quadrant = zeros(1,number_mod);
progress = waitbar(0, 'Modular Optmization...');
number_iter = 1;
last_quadrant = 0;



        



    for q =1:1:number_mod
        
       dist_to_center(q) = norm(modquad.quads(q).r_i_0);
       
       if (modquad.quads(q).r_i_0(1)>0) && (modquad.quads(q).r_i_0(2)>0)
           quadrant(q) = 1;
           
       elseif (modquad.quads(q).r_i_0(1)<0) && (modquad.quads(q).r_i_0(2)>0)
           quadrant(q) = 2;
           
       elseif (modquad.quads(q).r_i_0(1)<0) && (modquad.quads(q).r_i_0(2)<0)
           quadrant(q) = 3;
           
       elseif (modquad.quads(q).r_i_0(1)>0) && (modquad.quads(q).r_i_0(2)<0)
           quadrant(q) = 4;
       end
       
    end
    
    
for q =1:1:number_mod
    waitbar(q/number_mod, progress);
    [maxi,max_index] = max(dist_to_center);
    
    if (quadrant(max_index)==last_quadrant)
   
        for q_i =1:1:s(2)
            if (dist_to_center(q_i)==maxi) && (quadrant(q_i)~=last_quadrant)
                max_index = q_i;
            end
        end
    end

    waitbar(q/number_mod, progress); 
    
        for aux=1:1:2
            if aux==1
                angle_2(max_index) = pi/2;
            else
                angle_2(max_index) = 0;
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
        
        if error<f_min_alt
            f_min_alt = error;
            angle_min_alt(max_index) = angle_2(max_index);   
        end 
        end
        last_quadrant = quadrant(max_index);
        dist_to_center(max_index) = [];
        s = size(dist_to_center);

end
delete(progress);
end