%% File setup

addpath('plotting')
addpath('utils')
clear all
close all hidden
clc

%% Setup the structure

number_mod = 6;
shape = 'eucledian';
angle = zeros(1,number_mod); %initialize all modules to have same orientation
angle = pi/2*angle;
modquad = init_modquad(shape, number_mod, angle);
dt = 0.03;
t_final = 2*pi;
traj_type = 1;


[angle_min_py, f_min_py] = substuctureClass_min(dt,t_final,shape,number_mod,traj_type,modquad); %substructure optmization by modules classification given its distance to the center of mass
%[angle_min_bf,f_min_bf] = bruteForce_min(dt,t_final,shape,number_mod,traj_type); %brute force search
%[angle_min_alt,f_min_alt] = alt_local_min(dt,t_final,shape,number_mod,traj_type,modquad); %local sequential search
%[angle_min_alt2,f_min_alt2] = alt2_structOptm(dt,t_final,shape,number_mod,traj_type); %local sequential search+ prioritizing modules far from the center
%[angle_min_mod,f_min_mod] = substruct_min(dt,t_final,shape,number_mod,traj_type,modquad); %substructure optmization (4 structures always and divided per quadrant)



%angles = load('angles_min_bf10');

modquad = init_modquad(shape, number_mod, angle_min_py);
r_i_0 = modquad.r_i_0;
R_i0_0 = modquad.R_i0_0;
S = structureConfiguration(r_i_0,R_i0_0, number_mod);
S = double(vpa(S));

%% Simulate modquad motion

modquad.R_0_w = eye(3);
modquad.omega_0_0 = [0 0 0]';
modquad.x_0_w = [0 0 0]';
modquad.x_0_w_dot = [0 0 0]';
modquad.x_0_w_ddot = [0 0 0]';
dt = 0.03;
modquad.dt = dt;
t_final = 2*pi;

progress = waitbar(0, 'Simulating...');
modquad.wrenchSum_des_S = zeros(6,1);

modquad.pos_error =0;

try
for t = 0:dt:t_final
    waitbar(t/t_final, progress);
    modquad.t = t;
    modquad = trajectory(modquad,traj_type,dt);
    [modquad,flag] = PosControl_ForceDistrib(modquad,S);
    modquad = attControl_motorDistrib(modquad,dt);
    modquad = updateDeriv(modquad, dt);
    modquad = storeHistory(modquad, dt);
%     modquad = plot_modquad(modquad);
end
catch
%     modquad = plot_history(modquad);
    disp('Simulation Failed');
end
delete(progress);

%% Plotting

modquad = plot_history(modquad);

%% fmincon Optimizationf_min_opt = inf;
%{
f_min_opt = inf;
number_iter = 1;
lb = zeros(1,modquad.n);
ub = (pi/2) *ones(1, modquad.n);
Aeq = [1 0 0 0;
       1 -1 0 0;
       0 0 1 0;
       0 0 1 -1];
beq = [0;0;0;0];

options = optimoptions(@fmincon,'Display','iter','Algorithm','sqp-legacy','OptimalityTolerance',1e-20, 'StepTolerance',1e-20,'MaxFunctionEvaluations',1000);

for aux=1:1:number_iter
    
    [angle_min_local, f_min_local] = optmization(shape,number_mod,options,Aeq,beq,lb,ub,dt, t_final, traj_type);
    
    if f_min_local < f_min_opt 
        f_min_opt = f_min_local;
        angle_min_opt = angle_min_local;
    end
end
    
%}
%%
modquad = init_modquad(shape, number_mod, angle_min_bf);
r_i_0 = modquad.r_i_0;
R_i0_0 = modquad.R_i0_0;
S = structureConfiguration(r_i_0,R_i0_0, number_mod);
S = double(vpa(S));

%modquad = init_modquad(shape, number_mod, angle_min_bf);
%S = structureConfiguration(modquad);
%S = double(vpa(S));
%% Simulate modquad motion
modquad.R_0_w = eye(3);
modquad.omega_0_0 = [0 0 0]';
modquad.x_0_w = [0 0 0]';
modquad.x_0_w_dot = [0 0 0]';
modquad.x_0_w_ddot = [0 0 0]';
dt = 0.03;
modquad.dt = dt;
t_final =2*pi;
new_animation = 1;
progress = waitbar(0, 'Simulating...');
modquad.wrenchSum_des_S = zeros(6,1);
modquad.pos_error = zeros(3,1);
try
for t = 0:dt:t_final
    waitbar(t/t_final, progress);
    modquad.t = t;
    modquad = trajectory(modquad,traj_type,dt);
    [modquad,flag]  = PosControl_ForceDistrib(modquad,S);
    modquad = attControl_motorDistrib(modquad,dt);
    modquad = updateDeriv(modquad, dt);
    modquad = storeHistory(modquad, dt);
%     modquad = plot_modquad(modquad);
end
opt_error = modquad.pos_error;
catch
%     modquad = plot_history(modquad);
    disp('Simulation Failed');
end
delete(progress);

%% Plotting

modquad = plot_history(modquad);

