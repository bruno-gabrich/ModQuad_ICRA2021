function [angle_2, f_min_alt] = substruct_min(dt,t_final,shape,number_mod,traj_type,modquad)

f_min_alt = inf;
angle_min = zeros(1,number_mod);  
angle_2 = ones(1,number_mod);
angle_2 = (pi/2)*angle_2;
quadrant = zeros(1,number_mod);
progress = waitbar(0, 'Modular Optmization...');




    for q =1:1:number_mod
        
       
       
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

struct1 = find(abs(quadrant-1) < 0.001);
struct2 = find(abs(quadrant-2) < 0.001);
struct3 = find(abs(quadrant-3) < 0.001);
struct4 = find(abs(quadrant-4) < 0.001);


numberConfig1 = 2^size(struct1,2);
C1 = -(dec2bin(0:(numberConfig1)-1) - 49);
C1(C1==1)= pi/2;
C1= C1'; 

numberConfig2 = 2^size(struct2,2);
C2 = -(dec2bin(0:(numberConfig2)-1) - 49);
C2(C2==1)= pi/2;
C2= C2';

numberConfig3 = 2^size(struct3,2);
C3 = -(dec2bin(0:(numberConfig3)-1) - 49);
C3(C3==1)= pi/2;
C3= C3';

numberConfig4 = 2^size(struct4,2);
C4 = -(dec2bin(0:(numberConfig4)-1) - 49);
C4(C4==1)= pi/2;
C4= C4';





    
 for count1=1:1:numberConfig1
    waitbar(count1/(numberConfig1+numberConfig2+numberConfig3+numberConfig4), progress);
    for c2=1:1:size(struct1,2)
        angle_2(struct1(c2)) = C1(c2,count1);
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
            for q=1:1:size(struct1,2)
            angle_min(struct1(q)) = angle_2(struct1(q)); 
            end
        end 
 end
f_min_alt = inf; 
for q=1:1:size(struct1,2)
angle_2(struct1(q)) = angle_min(struct1(q)); 
end

for count3=1:1:numberConfig3
    waitbar((count1 + count3)/(numberConfig1+numberConfig2+numberConfig3+numberConfig4), progress);
    for c2=1:1:size(struct3,2)
        angle_2(struct3(c2)) = C3(c2,count3);
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
            for q=1:1:size(struct3,2)
            angle_min(struct3(q)) = angle_2(struct3(q)); 
            end
        end 
end
 
%f_min_alt = inf;
for q=1:1:size(struct3,2)
angle_2(struct3(q)) = angle_min(struct3(q)); 
end

for count2=1:1:numberConfig2
     waitbar((count1 + count2 +count3)/(numberConfig1+numberConfig2+numberConfig3+numberConfig4), progress);
    for c2=1:1:size(struct2,2)
        angle_2(struct2(c2)) = C2(c2,count2);
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
            for q=1:1:size(struct2,2)
            angle_min(struct2(q)) = angle_2(struct2(q)); 
            end
        end 
end
f_min_alt = inf;
for q=1:1:size(struct2,2)
angle_2(struct2(q)) = angle_min(struct2(q)); 
end




for count4=1:1:numberConfig4
    waitbar((count1 + count2 + count3 + count4)/(numberConfig1+numberConfig2+numberConfig3+numberConfig4), progress);
    for c2=1:1:size(struct4,2)
        angle_2(struct4(c2)) = C4(c2,count4);
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
            for q=1:1:size(struct4,2)
            angle_min(struct4(q)) = angle_2(struct4(q)); 
            end
        end 
end
 
for q=1:1:size(struct4,2)
angle_2(struct4(q)) = angle_min(struct4(q)); 
end
delete(progress);
end
       


