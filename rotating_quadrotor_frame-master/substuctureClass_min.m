function [angle_min, f_min] = substuctureClass_min(dt,t_final,shape,number_mod,traj_type,modquad)

e_min = inf;
f_min = inf;
angle_min = zeros(1,number_mod);  
angle = zeros(1,number_mod);
angle = (pi/2)*angle;
mod2center = zeros(1,number_mod);
n=number_mod;


for i =1:1:n
   mod2center(i) = norm(modquad.quads(i).r_i_0);
end
    
mod2center = unique(mod2center);
SubGroups = zeros(number_mod,size(mod2center,2));
numberSubGroups = size(mod2center,2);

%for j=1:1:numberSubGroups
for j=numberSubGroups:-1:1    
        for i=1:1:n
            if mod2center(j)==norm(modquad.quads(i).r_i_0)
               SubGroups(i,j) = i;
            end
        end
end
    
%for j=1:1:numberSubGroups
for j=numberSubGroups:-1:1
    
    SubGroup = SubGroups(:,j);
    SubGroup = SubGroup(SubGroup~=0);
    elemSubGroup = size(SubGroup,1);
    h = linspace(1, elemSubGroup, elemSubGroup);
    numberConfig = 2^elemSubGroup;
    C = -(dec2bin(0:(numberConfig)-1) - 49);
    C(C==1)= pi/2;
    C= C';
    
progress = waitbar(0, 'Modular Optmization');    
 for q=1:1:numberConfig
    
     waitbar(q/numberConfig, progress)
     angle_min(SubGroup(h)) = C(h,q);
 
  
        modquad = init_modquad(shape,number_mod,angle_min);
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
        e = (pos_error*dt);
        
        pos_error = modquad.pos_error;
        f_vector = pinv(S_new) * modquad.wrenchSum_des_S * dt;
        modquad.f = norm(f_vector);
        f = modquad.f;
        error = (pos_error*dt);
        pos_error = 0;
        if rank(S_new)>=6
            
            if f<f_min
                f_min = f;
                angle(SubGroup(h)) = angle_min(SubGroup(h));    
            end 
        end
 end
     angle_min = angle;

end
delete(progress);
end