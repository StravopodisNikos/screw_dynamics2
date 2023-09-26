function [dJb_ij_dt] = calculateFirstTimeDerivativeBodyJacobian1_Mueller(Jb,dq,USE_SYM)
% Implements the FIRST (=) of eq.(8) in Mueller Dynamics
% Jb_ij, where i denotes the body of the manipulator the Body Jacobian
% relates to and j the current joint variable counter

nDoF = size(Jb,2);
i = nDoF; % assumes the Jacobian relates to the {T} frame after the last active twist

if (USE_SYM)
    dJb_ij = sym(zeros(6,nDoF),'d');
else
    dJb_ij = zeros(6,nDoF);
end

for j=1:i
    if (USE_SYM)
        dJ = sym(zeros(6,1),'d');
    else
        dJ = zeros(6,1);
    end
    for k=j+1:i
        dJ = dJ + liebracket_426_new(Jb(:,j,4), Jb(:,k,4)) * dq(k); % For the Jacobian that relates to the {T} frame
    end
    dJb_ij(:,j) = dJ;
end

dJb_ij_dt = dJb_ij;
end