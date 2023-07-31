function [Js] = calculateSpatialJacobians2_Mueller(Ci, Ai, YI_i,nDoF)

% [6-2-23]
% Calculates the SECOND (=) in eq.(28) in Mueller Kinematics Paper

for i=1:nDoF
    Js_j(:,i) = ad(Ci(:,:,i)*inv(Ai(:,:,i))) * YI_i(:,i);
end

Js = Js_j;
end