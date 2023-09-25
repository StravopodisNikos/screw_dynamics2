function [Jb1_bodies] = calculateBodyJacobians1_Mueller(Ci, XI_ii,nBodies,nDoF,USE_SYM)
% [6-2-2022]
% Calculates the Jacobians derived from the FIRST (=) of eq.(16) in
% Mueller Kinematics

% Jb1_bodies = [Jb_1j_1 | Jb_2j_1 | ... | Jb_nj_1 | Jb_tj_1] for a serial chain with n
% bodies, the last Jacobian relates to the origin of the {T} frame! If the
% {T} frame originates to the last twist then Jb_nj_1==Jb_tj_1 !

% From the above jacobias Jb_tj_1 is the Body Manipulator Jacobian
% presented in the Murray Book!
if (USE_SYM)
    Jb1_bodies = sym(zeros(6,nDoF,nBodies),'r');
else
    Jb1_bodies = zeros(6,nDoF,nBodies);
end

for nbod_cnt=1:nBodies
    if (USE_SYM)
        for j=1:nDoF
            Jb1_bodies(:,j,nbod_cnt) = simplify( ad( inv(Ci(:,:,nbod_cnt)) * Ci(:,:,j)  ) * XI_ii(:,j));
        end
    else
        for j=1:nDoF
            Jb1_bodies(:,j,nbod_cnt) = ad( inv(Ci(:,:,nbod_cnt)) * Ci(:,:,j)  ) * XI_ii(:,j);
        end        
    end
end

end
