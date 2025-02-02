function f = obj_fun(x,number_mod)
   
%{    
    for q=1:1:number_mod
       
        angle_3(q) = x(q);
    end
    
    modquad = init_modquad(shape, number_mod, angle_3);
    S = structureConfiguration(modquad);
    S_new = double(vpa(S));
    
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
        modquad = PosControl_ForceDistrib(modquad,S_new);
        modquad = attControl_motorDistrib(modquad,dt);
        modquad = updateDeriv(modquad, dt);
    end

   
    %f_vector = pinv(S_new) * modquad.wrenchSum_des_S * dt;
    %f = norm(f_vector);
    f = modquad.pos_error*dt;
%}
       for q =1:1:3*number_mod
           fun(q) = x(q);
       end
       
       f = norm(fun);
           
end