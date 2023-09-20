function [dSi_dt] = calculateFirstTimeDerivativeScrew_Sanchez(i,Js,dq)
% Implements eq.(9) in paper:
% "The differential calculus of screws: theory, geometrical interpretation
% and applications", Sanchez 2008
% these are also the columns of the 1st derivative of the Spatial Jacobian
% calculated by calculateFirstTimeDerivativeSpatialJacobian1_Mueller.m
dS_i = sym(zeros(6,1),'d');

for j=1:i-1
    dS_i = dS_i + dq(j) * liebracket_426_new2(Js(:,j), Js(:,i));
end

dSi_dt = dS_i;

% Verified in [20-9-23], in analysis_anthropomorphic_3dof_sym.m
end