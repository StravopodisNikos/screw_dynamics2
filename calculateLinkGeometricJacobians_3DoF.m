function [Jgl] = calculateLinkGeometricJacobians_3DoF(q, xi_ai, g_sai0, g_sli0)
% [18-3-24] Developed for analytical computation of gravity vector
% Based on Par.7.1.1, p.252,eq.(7.16,7.18), Siciliano Book

Jgl = zeros(3,3,3);
O31 = zeros(3,1);
% Extract the T^0_i matrices
T_01 = twistexp(xi_ai(:,1),q(1)) * g_sai0(:,:,1);
T_02 = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * g_sai0(:,:,2);
T_03 = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * twistexp(xi_ai(:,3),q(3)) * g_sai0(:,:,3);

Tl_01 = twistexp(xi_ai(:,1),q(1)) * g_sli0(:,:,1);
Tl_02 = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * g_sli0(:,:,2);
Tl_03 = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * twistexp(xi_ai(:,3),q(3)) * g_sli0(:,:,3);

z_01 = T_01(1:3,3); % local z-axis is the rotation axis
z_02 = T_02(1:3,1); % local x-axis is the rotation axis
z_03 = T_03(1:3,1); % local x-axis is the rotation axis

% p_00 = zeros(3,1);
p_01 = T_01(1:3,4);
p_02 = T_02(1:3,4);
p_03 = T_03(1:3,4);

pl_01 = Tl_01(1:3,4);
pl_02 = Tl_02(1:3,4);
pl_03 = Tl_03(1:3,4);

Jp1_l1 =  cross(z_01, (pl_01 - p_01) );
Jgl(:,:,1) = [Jp1_l1 O31 O31];

Jp1_l2 =  cross(z_01, (pl_02 - p_01) );
Jp2_l2 =  cross(z_02, (pl_02 - p_02) );
Jgl(:,:,2) = [Jp1_l2 Jp2_l2 O31];

Jp1_l3 =  cross(z_01, (pl_03 - p_01) );
Jp2_l3 =  cross(z_02, (pl_03 - p_02) );
Jp3_l3 =  cross(z_03, (pl_03 - p_03) );
Jgl(:,:,3) = [Jp1_l3 Jp2_l3 Jp3_l3];

end