function [dJs_j1_dt, dJs_j2_dt, dJs_j3_dt] = calculateFirstTimeDerivativeSpatialJacobians_Mueller(Js,Vs,dq)
% Executes the 3 "=" of eq.17 in Mueller/Dynamics
% Input: 1. Js as calculated by Mueller eq.28/Kinematics
%        2. Vs as calculated by Mueller eq.30/Kinematics

nDoF = size(Js,2);

% Implements the FIRST (=) of eq.(17)
dJs_j1_dt = sym(zeros(6,nDoF),'d');
for j=1:nDoF
    dJ1 = sym(zeros(6,1),'d');
    for k=1:j
        dJ1 = dJ1 + liebracket_426_new2(Js(:,k), Js(:,j)) * dq(k);
    end
    dJs_j1_dt(:,j) = dJ1;
end
% Implements the SECOND (=) of eq.(17)
dJs_j2_dt = sym(zeros(6,nDoF),'d');
for j=1:nDoF
    SJ = sym(zeros(6,1),'d');
    for k=1:j
        SJ = SJ + Js(:,k);
    end
    dJs_j2_dt(:,j) = liebracket_426_new2(SJ,Js(:,j)) * dq(k);
end
% Implements the THIRD (=) of eq.(17)
dJs_j3_dt = sym(zeros(6,nDoF),'d');
for j=1:nDoF
    dJs_j3_dt(:,j) = liebracket_426_new2(Vs(:,j), Js(:,j));
end

end