function [g_q] = calculateGravityVectorArithmetic_mod(qold, Uold, Ufwd, q, xi_ai,gsli0, Mi, nDoF ,nD, vert_axis)
%% Modified arithmetic calculation, using higher order finite diffs
%% Second order central finite difference method is implemented
g_q = zeros(3,1);
tol_dq = 0.01;
tol_dU = 0.000001;
step_q_min = 0.1;
Delta_q = q - qold;
% Get the current energy
% [U_q,~] = calculatePotentialEnergyArithmetic(q, xi_ai, gsli0, Mi, nDoF ,nD, vert_axis);


%% Try 1
% % Delta_U =  Ufwd - Uold;
% % g_q(1) = Delta_U(1) / Delta_q(1);
% % g_q(2) = Delta_U(2) / Delta_q(2);
% % if abs(Delta_U(3)) > tol_dU
% %     g_q(3,1) = ( Delta_U(3) / Delta_q(3) );
% % else
% %     g_q(3,1) = Delta_U(3);
% % end
%% Try 2
Delta_U =  Ufwd - Uold;
g_q(1,1) = Delta_U(1) / Delta_q(1);
g_q(2,1) = Delta_U(2) / Delta_q(2);
g_q(3,1) = Delta_U(3) / Delta_q(3);
end