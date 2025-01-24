function S = structureConfiguration(r_i_0,R_i0_0, number_mod)

    
   % R_i_s = sym('R_i_s%d%d', [3 3*modquad.n]);
    A = zeros(3*number_mod,3*number_mod);
   % P_i_R_i_s = sym('P_i_R_i_s%d%d', [3 3*modquad.n]);

    R_i_s = zeros(3,3*number_mod);
    P_i_R_i_s = zeros(3,3*number_mod);
%    q1=linspace(1,number_mod,number_mod);
    for q = 1:number_mod
        
        A(3*(q-1)+1:(3*(q-1)+1)+2,3*(q-1)+1:(3*(q-1)+1)+2) = [0 0 0;0 1 0;0 0 1];
        %R_i_s(:,3*(q-1)+1:(3*(q-1)+1)+2) = modquad.quads(q).R_i0_0;
        R_i_s(:,3*(q-1)+1:(3*(q-1)+1)+2) = R_i0_0(:,:,q);
        %P_i_R_i_s(:,3*(q-1)+1:(3*(q-1)+1)+2) = makeskew(modquad.quads(q).r_i_0) * modquad.quads(q).R_i0_0;
        P_i_R_i_s(:,3*(q-1)+1:(3*(q-1)+1)+2) = makeskew(r_i_0(:,q)) * R_i0_0(:,:,q);
    end
    
%    R_i_s(:,3*(q1-1)+1:(3*(q1-1)+1)+2) = R_i0_0(:,:,q1);
%    P_i_R_i_s(:,3*(q1-1)+1:(3*(q1-1)+1)+2) = makeskew(r_i_0(:,q1)) * R_i0_0(:,:,q1);
    
    S_aux = [R_i_s;P_i_R_i_s];
    S = S_aux * A;
end
    

  