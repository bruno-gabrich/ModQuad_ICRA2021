function modquad = storeHistory(modquad, dt)

    %% Pushback the history
    if ~isfield(modquad.history, 'alpha')
        modquad.history.alpha = zeros(modquad.n, 1);
    end

    for q = 1:modquad.n
        modquad.history.alpha(q,end+1) = modquad.quads(q).alpha;
        modquad.history.quads(q).R_i_w(:,:,end+1) = modquad.quads(q).R_i_w;
        modquad.history.quads(q).x_i_w(:,end+1) = modquad.quads(q).x_i_w;
        modquad.history.quads(q).thrust = modquad.quads(q).thrust;
        modquad.history.quads(q).f1 = modquad.quads(q).f1;
        modquad.history.quads(q).f2 = modquad.quads(q).f2;
        modquad.history.quads(q).f3 = modquad.quads(q).f3;
        modquad.history.quads(q).f4 = modquad.quads(q).f4;
    end        
    modquad.history.x_0_w(:,end+1) = modquad.x_0_w;
    modquad.history.R_0_w(:,:,end+1) = modquad.R_0_w;
    modquad.history.eul_0_w(:,end+1) = radtodeg(rotm2eul(modquad.R_0_w,'XYZ'));
    modquad.history.t(end+1) = modquad.history.t(end) + dt;
    
    modquad.history.des.x_0_w(:,end+1) = modquad.des.x_0_w;
    modquad.history.des.R_0_w(:,:,end+1) = modquad.des.R_0_w;
    modquad.history.des.eul_0_w(:,end+1) = radtodeg(rotm2eul(modquad.des.R_0_w,'XYZ'));
end
