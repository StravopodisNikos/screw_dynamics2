function [g_q,U_q] = calculateGravityVectorArithmetic(qold, Uold, q, xi_ai,gsli0, Mi, nDoF ,nD, vert_axis)
g_q = zeros(3,1);
% Get the deltas for joint positions
% Delta_q(1) = q(1) - qold(1);
% Delta_q(2) = q(2) - qold(2);
% Delta_q(3) = q(3) - qold(3);
Delta_q = q - qold;
% Get the new potential energy
[U_q,~] = calculatePotentialEnergyArithmetic(q, xi_ai, gsli0, Mi, nDoF ,nD, vert_axis);

Delta_U = U_q - Uold;
% Delta_U_num = sum(U_q) - sum(Uold);
tol_dq = 0.0000001;
if abs(Delta_q(1)) > tol_dq
    g_q(1,1) = ( Delta_U(1) / Delta_q(1) );
else
    g_q(1,1) = 0;
end
if abs(Delta_q(2)) > tol_dq
    g_q(2,1) = ( Delta_U(2) / Delta_q(2) );
else
    g_q(2,1) = 0;
end
if abs(Delta_q(3)) > tol_dq
    g_q(3,1) = ( Delta_U(3) / Delta_q(3) );
else
    g_q(3,1) = 0;
end

end