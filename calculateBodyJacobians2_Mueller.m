function [Jb2_bodies] = calculateBodyJacobians2_Mueller(Ci, Ai,  YI_ii,nBodies,nDoF)
% [6-2-2023]
% Calculates the Jacobians derived from the SECOND (=) of eq.(16) in
% Mueller Kinematics

% Jb2_bodies = [Jb_1j_2 | Jb_2j_2 | ... | Jb_nj_2 | Jb_tj_2] for a serial chain with n
% bodies, the last Jacobian relates to the origin of the {T} frame! If the
% {T} frame originates to the last twist then Jb_nj_1==Jb_tj_1 !

% YI_ii = xi_ai are the spatial twists also presented by Murray Book!

% From the above jacobias Jb_tj_2 is the Body Manipulator Jacobian
% presented in the Murray Book!

Jb2_bodies = sym(zeros(6,nDoF,nBodies),'r');
T = sym(zeros(4,4,nDoF),'r');

for nbod_cnt=1:nBodies
    for j=1:nDoF
        T(:,:,j) = calculateTmatrix(Ci(:,:,j), Ai(:,:,j));
        Jb2_bodies(:,j,nbod_cnt) = simplify(  ad( inv( T(:,:,j) \ Ci(:,:,nbod_cnt) ) ) * YI_ii(:,j)  );    
    end
end

% Jb2_bodies == Jb1_bodies calculated in calculateBodyJacobians1_Mueller!
end