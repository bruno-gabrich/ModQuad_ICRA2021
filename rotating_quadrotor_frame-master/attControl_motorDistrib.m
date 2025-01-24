function modquad = attControl_motorDistrib(modquad,dt)

kp = 0.2;
kd = 0.002;
w = 0.1;


y1 = 1;
y2 = 1;
y3 = 1;
y4 = 1;

modGeomMatrix = [1 1 1 1;-y1 -y2 y3 y4];
D = pinv(modGeomMatrix);

%% Filter to derive alpha_dot



    for q=1:1:modquad.n

        modquad.quads(q).alpha_dot_raw = (modquad.quads(q).alpha - modquad.quads(q).alpha_old) / dt;
        modquad.quads(q).alpha_dot = w * modquad.quads(q).alpha_dot_raw + (1-w) * modquad.quads(q).alpha_dot_raw_old;
        
%% Attitude Controller

        alpha_ddot = kp * (modquad.quads(q).alpha - modquad.quads(q).alpha_old) + kd * (modquad.quads(q).alpha_dot - modquad.quads(q).alpha_dot_old);

%% Control Mixer
        
        u = D * [modquad.quads(q).thrust; modquad.I_0(1,1) * alpha_ddot];
        
  %      modquad.quads(q).f1 = (1/4) * modquad.quads(q).thrust - (1/4) * alpha_ddot;
  %      modquad.quads(q).f2 = (1/4) * modquad.quads(q).thrust - (1/4) * alpha_ddot;
  %      modquad.quads(q).f3 = (1/4) * modquad.quads(q).thrust + (1/4) * alpha_ddot;
  %      modquad.quads(q).f4 = (1/4) * modquad.quads(q).thrust + (1/4) * alpha_ddot;
    
   %     if modquad.quads(q).f1 <=0
   %         modquad.quads(q).f1 = 0;
   %     end
        
   %     if modquad.quads(q).f2 <=0
   %         modquad.quads(q).f2 = 0;
   %     end
        
   %     if modquad.quads(q).f3 <=0
   %         modquad.quads(q).f3 = 0;
   %     end
        
   %     if modquad.quads(q).f4 <=0
   %         modquad.quads(q).f4 = 0;
   %     end
        
        modquad.quads(q).f1 = u(1);  
        modquad.quads(q).f2 = u(2);
        modquad.quads(q).f3 = u(3);
        modquad.quads(q).f4 = u(4);
        
        modquad.quads(q).alpha_dot_old = modquad.quads(q).alpha_dot;
        modquad.quads(q).alpha_dot_raw_old = modquad.quads(q).alpha_dot_raw;
        modquad.quads(q).alpha_old = modquad.quads(q).alpha;

      
    end
    
    
    
