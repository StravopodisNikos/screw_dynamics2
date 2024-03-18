function [g_sp] = calculateGravityVectorAnalyticalBody(q, xi_ai, gsli0, Jbg, mi)
% [18-3-24] Jbg is produced in calculateLinkCoMJacobians_3DoF. It is the Jbsli.
% Spatial Jacobian doesnt help here.

g_earth = -9.80665; % [m/s^2]
mg = zeros(3,1);
mg = g_earth .* mi;
g = [0 0 g_earth]';
for i=1:3
    exp_ai(:,:,i) = twistexp( xi_ai(:,i),q(i) );
end
gsli(:,:,1) = exp_ai(:,:,1) * gsli0(:,:,1);
gsli(:,:,2) = exp_ai(:,:,1) * exp_ai(:,:,2) *gsli0(:,:,2);
gsli(:,:,3) = exp_ai(:,:,1) * exp_ai(:,:,2) * exp_ai(:,:,3) *gsli0(:,:,3);

% g_sp(1,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,1,1) + mi(2) * g' * gsli(1:3,1:3,2)' * Jbg(:,1,2) + mi(3) * g' * gsli(1:3,1:3,3)' * Jbg(:,1,3) );
% g_sp(2,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,2,1) + mi(2) * g' * gsli(1:3,1:3,2)' * Jbg(:,2,2) + mi(3) * g' * gsli(1:3,1:3,3)' * Jbg(:,2,3) );
% g_sp(3,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,3,1) + mi(2) * g' * gsli(1:3,1:3,2)' * Jbg(:,3,2) + mi(3) * g' * gsli(1:3,1:3,3)' * Jbg(:,3,3) );
% g_sp(1,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,1,1) ); % worked
% g_sp(2,1) = - ( mi(2) * g' * gsli(1:3,1:3,2)' * Jbg(:,2,2) ); % worked
% g_sp(3,1) = - ( mi(3) * g' * gsli(1:3,1:3,3)' * Jbg(:,3,3) ); % worked
g_sp(1,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,1,1) ); % best
g_sp(2,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,2,1) + mi(2) * g' * gsli(1:3,1:3,2)' * Jbg(:,2,2) ); % best
g_sp(3,1) = - ( mi(1) * g' * gsli(1:3,1:3,1)' * Jbg(:,3,1) + mi(2) * g' * gsli(1:3,1:3,2)' * Jbg(:,3,2) + mi(3) * g' * gsli(1:3,1:3,3)' * Jbg(:,3,3) ); % best
end