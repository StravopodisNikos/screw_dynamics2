function [g_q,U_q] = calculateGravityVectorArithmetic(qold, Uold, q, xi_ai,gsli0, Mi, nDoF ,nD, vert_axis)

% Get the deltas for joint positions
Delta_q(1) = q(1) - qold(1);
Delta_q(2) = q(2) - qold(2);
Delta_q(3) = q(3) - qold(3);

% Get the new potential energy
[U_q,~] = calculatePotentialEnergyArithmetic(q, xi_ai, gsli0, Mi, nDoF ,nD, vert_axis);
Delta_U = U_q - Uold;

if abs(Delta_q(1)) > 0.0001
    g_q(1) = Delta_U / Delta_q(2);
else
    g_q(1) = 0;
end
if abs(Delta_q(2)) > 0.0001
    g_q(2) = Delta_U / Delta_q(2);
else
    g_q(2) = 0;
end
if abs(Delta_q(3)) > 0.0001
    g_q(3) = Delta_U / Delta_q(3);
else
    g_q(3) = 0;
end

end