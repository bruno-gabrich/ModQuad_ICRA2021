function [angle_min_local, f_min_local] = optmization(shape,number_mod,options,Aeq,beq,lb,ub,dt,t_final,traj_type)




angle_min_local = zeros(1,number_mod);
x0 = ones(1,number_mod);


for aux=1:1:number_mod
    x0(1,aux) = (pi/2) * rand(1,1);
end

  
[angle_min_local,f_min_local] = fmincon(@(x) obj_fun(x,shape,number_mod,dt,t_final,traj_type),x0,[],[],[],[],lb,ub,[],options);


    
