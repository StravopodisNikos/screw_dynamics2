function [dJs_j1_dt, dJs_j2_dt, dJs_j3_dt] = calculateFirstTimeDerivativeSpatialJacobians_Mueller(Js,Vs,dq,USE_SYM)
% Executes the 3 "=" of eq.17 in Mueller/Dynamics
% Input: 1. Js as calculated by Mueller eq.28/Kinematics
%        2. Vs as calculated by Mueller eq.30/Kinematics
% Updates:
% [26-9-23] - only first "=" output considered

nDoF = size(Js,2);

if (USE_SYM) 
    dJs_j1_dt = sym(zeros(6,nDoF),'d');
    dJs_j2_dt = sym(zeros(6,nDoF),'d');
    dJs_j3_dt = sym(zeros(6,nDoF),'d');
else
    dJs_j1_dt = zeros(6,nDoF);
    dJs_j2_dt = zeros(6,nDoF);
    dJs_j3_dt = zeros(6,nDoF);
end

% Implements the FIRST (=) of eq.(17)
for j=1:nDoF
    if (USE_SYM)
        dJ1 = sym(zeros(6,1),'d');
    else
        dJ1 = zeros(6,1);
    end
    for k=1:j
        dJ1 = dJ1 + liebracket_426_new(Js(:,k), Js(:,j)) * dq(k);
    end
    dJs_j1_dt(:,j) = dJ1;
end
% Implements the SECOND (=) of eq.(17) -> WRONG ANSWER
for j=1:nDoF
    if (USE_SYM)
        SJ = sym(zeros(6,1),'d');
    else
        SJ = zeros(6,1);
    end
    for k=1:j
        SJ = SJ + Js(:,k);
    end
    dJs_j2_dt(:,j) = liebracket_426_new(SJ,Js(:,j)) * dq(k);
end
% Implements the THIRD (=) of eq.(17)
% Removed because i need only the last velocity twist, so Vs is 6x1 only,
% despite that, with previous version it works fine
% for j=1:nDoF
%     dJs_j3_dt(:,j) = liebracket_426_new2(Vs(:,j), Js(:,j));
% end

end