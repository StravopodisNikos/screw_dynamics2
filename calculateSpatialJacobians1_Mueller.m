function [Js] = calculateSpatialJacobians1_Mueller(Ci, XI_ii,nDoF)

% [6-2-23]
% Calculates the FIRST (=) in eq.(28) in Mueller Kinematics Paper

for i=1:nDoF
    Js_j(:,i) = ad(Ci(:,:,i)) * XI_ii(:,i);
end

Js = Js_j;
end