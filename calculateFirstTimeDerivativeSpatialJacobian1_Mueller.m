function [dJs_j_dt] = calculateFirstTimeDerivativeSpatialJacobian1_Mueller(Js,dq)
% Implements the FIRST (=) of eq.(17) in Mueller Dynamics
% Jb_ij, where i denotes the body of the manipulator the Body Jacobian
% relates to and j the current joint variable counter

nDoF = size(Js,2);

dJs_j = sym(zeros(6,nDoF),'d');
for j=1:nDoF
    dJ = sym(zeros(6,1),'d');
    for k=1:j
        dJ = dJ + liebracket_426_new2(Js(:,k), Js(:,j)) * dq(k);
    end
    dJs_j(:,j) = dJ;
end

dJs_j_dt = dJs_j;
end