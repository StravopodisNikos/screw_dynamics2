function [g_bd] = calculateGravityVectorAnalyticalBody(q, xi_ai, gsli0, Jbg, mi)
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

% % It is VERY IMPORTANT to set the orientation of the COM tf!
% g_bd = - Jbg(:,:,1)' * ( gsli(1:3,1:3,1) * mi(1) * g ) - Jbg(:,:,2)' *( gsli(1:3,1:3,2) * mi(2) * g ) - Jbg(:,:,3)' *( gsli(1:3,1:3,3) * mi(3) * g ) ;
g_bd(1,1) = - Jbg(:,1,1)' * ( gsli(1:3,1:3,1) * mi(1) * g );
g_bd(2,1) = - Jbg(:,2,1)' * ( gsli(1:3,1:3,1) * mi(1) * g ) - Jbg(:,2,2)' * ( gsli(1:3,1:3,2) * mi(2) * g ) - Jbg(:,2,3)' * ( gsli(1:3,1:3,3) * mi(3) * g );
% g_bd(3,1) = - Jbg(:,3,3)' * ( gsli(1:3,1:3,3) * mi(3) * g ); % must be always the same as below because columns used are 0!
g_bd(3,1) = - Jbg(:,3,1)' * ( gsli(1:3,1:3,1) * mi(1) * g ) - Jbg(:,3,2)' * ( gsli(1:3,1:3,2) * mi(2) * g ) - Jbg(:,3,3)' * ( gsli(1:3,1:3,3) * mi(3) * g );

% g_bd_1_1 = - Jbg(:,1,1)' * ( gsli(1:3,1:3,1) * (mi(1)) * g );
% g_bd(1,1) = g_bd_1_1;
% g_bd_2_1 = - Jbg(:,2,1)' * ( gsli(1:3,1:3,1) * (mi(1)) * g ) - Jbg(:,2,2)' * ( gsli(1:3,1:3,2) * (mi(2)) * g ) - Jbg(:,2,3)' * ( gsli(1:3,1:3,3) * (mi(3)) * g );
% g_bd(2,1) = g_bd_2_1;
% g_bd_3_1 = - Jbg(:,3,1)' * ( gsli(1:3,1:3,1) * (mi(1)) * g ) - Jbg(:,3,2)' * ( gsli(1:3,1:3,2) * (mi(2)) * g ) - Jbg(:,3,3)' * ( gsli(1:3,1:3,3) * (mi(3)) * g );
% g_bd(3,1) = g_bd_3_1;
end