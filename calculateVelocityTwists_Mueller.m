function [Vst_s_mul,Vst_b_mul] = calculateVelocityTwists_Mueller(Js,Jb,dq,ndof,USE_SYM)
% Executes eq.30 in Mueller/Kinematics paper.
% Input is the spatial jacobians as calculated by mueller

if (USE_SYM)
    V_i_s = sym(zeros(6,ndof),'d');
    V_i_b = sym(zeros(6,ndof),'d');
else
    V_i_s = zeros(6,ndof);
    V_i_b = zeros(6,ndof);
end

V_i_s(:,1) = Js(:,1) * dq(1);
for i=2:ndof
        V_i_s(:,i) = V_i_s(:,i-1) + Js(:,i) * dq(i); 
%     V_i_s(:,i) = Js(:,i) * dq(i);
end

% for i=1:ndof
%        V_i_b(:,i) = Jb(:,i,4) * dq(i); % {T} frame, that is the '4' Body Jacobian
% end

Vst_s_mul = Js * dq;
Vst_b_mul = Jb * dq;
end