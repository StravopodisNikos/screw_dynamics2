function [g_q] = calculateGravityVectorAnalytical(Jgl,mi)
% Jgl is produced in calculateLinkGeometricJacobians_3DoF

g_earth = -9.80665; % [m/s^2]
g = zeros(3,1);
g(3) = g_earth;

g_q(1,1) = - ( mi(1) * g' * Jgl(:,1,1) + mi(2) * g' * Jgl(:,1,2) + mi(3) * g' * Jgl(:,1,3) ); % (l_i,j): (1,1) (1,2) (1,3)
g_q(2,1) = - ( mi(1) * g' * Jgl(:,2,1) + mi(2) * g' * Jgl(:,2,2) + mi(3) * g' * Jgl(:,2,3) ); % (l_i,j): (2,1) (2,2) (2,3)
g_q(3,1) = - ( mi(1) * g' * Jgl(:,3,1) + mi(2) * g' * Jgl(:,3,2) + mi(3) * g' * Jgl(:,3,3) ); % (l_i,j): (3,1) (3,2) (3,3)
end