function [dJb_ij_dt] = calculateFirstTimeDerivativeBodyJacobian1_Mueller(Jb,dq)
% Implements the FIRST (=) of eq.(8) in Mueller Dynamics
% Jb_ij, where i denotes the body of the manipulator the Body Jacobian
% relates to and j the current joint variable counter

nDoF = size(Jb,2);
i = nDoF; % assumes the Jacobian relates to the {T} frame after the last active twist

dJb_ij = sym(zeros(6,nDoF),'d');

for j=1:i
    dJ = sym(zeros(6,1),'d');
    for k=j+1:i
        dJ = dJ + liebracket_426_new(Jb(:,j), Jb(:,k)) * dq(k);
    end
    
    dJb_ij(:,j) = dJ;
end

dJb_ij_dt = dJb_ij;
end