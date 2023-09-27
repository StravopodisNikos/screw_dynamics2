function [Jbsli,Jbsli_427] = Jbody_CoM_3DoF_sym(xi_ai, q, gsli0, i, USE_SYM)
% [26-9-23] No bug found, both answers the same, check is ok
% [27-9-23] Added USE_SYM for integration in numeric calculations
%           Inserted exp_ai computation in the function

if (USE_SYM)
    Jbsli = sym(zeros(6,3),'r');
    Jbsli_427 = sym(zeros(6,3),'r');
else
    Jbsli = zeros(6,3);
    Jbsli_427 = zeros(6,3);    
end

% Generate the exponentials
exp_ai(:,:,1) = twistexp(xi_ai(:,1),q(1));
exp_ai(:,:,2) = twistexp(xi_ai(:,2),q(2));
exp_ai(:,:,3) = twistexp(xi_ai(:,3),q(3));

% Based on eq. p.168
if i==1
    Jbsli(:,1) = inv(ad(exp_ai(:,:,1)*gsli0(:,:,1)))*xi_ai(:,1);
elseif i==2
    Jbsli(:,1) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*gsli0(:,:,2)))*xi_ai(:,1);
    Jbsli(:,2) = inv(ad(exp_ai(:,:,2)*gsli0(:,:,2)))*xi_ai(:,2);
elseif i==3
    Jbsli(:,1) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*gsli0(:,:,3)))*xi_ai(:,1);
    Jbsli(:,2) = inv(ad(exp_ai(:,:,2)*exp_ai(:,:,3)*gsli0(:,:,3)))*xi_ai(:,2);
    Jbsli(:,3) = inv(ad(exp_ai(:,:,3)*gsli0(:,:,3)))*xi_ai(:,3);
end

% Based on eq.4.27 p.176
Pi(:,:,1) = eye(4); Pi(:,:,2) = eye(4);% dummy

if i == 1
    [Aij_427] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, i, 1);
    Jbsli_427(:,1) = ad(inv(gsli0(:,:,i)))*Aij_427*xi_ai(:,1);
elseif i == 2
    for j=1:i
        [Aij_427] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, i, j);
        Jbsli_427(:,j) = ad(inv(gsli0(:,:,i)))*Aij_427*xi_ai(:,j);        
    end
elseif i == 3
    for j=1:i
        [Aij_427] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, i, j);
        Jbsli_427(:,j) = ad(inv(gsli0(:,:,i)))*Aij_427*xi_ai(:,j);        
    end        
end
    
% Jbsli_error = Jbsli-Jbsli_427;
end