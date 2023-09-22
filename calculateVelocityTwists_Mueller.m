function [V_i_s] = calculateVelocityTwists_Mueller(Js,dq,ndof)
% Executes eq.30 in Mueller/Kinematics paper.
% Input is the spatial jacobians as calculated by mueller

V_i_s = sym(zeros(6,3),'d');

V_i_s(:,1) = Js(:,1) * dq(1);
for i=2:ndof
    V_i_s(:,i) = V_i_s(:,i-1) + Js(:,i) * dq(i);
end

end