function [Ul,error_code] = calculatePotentialEnergyArithmetic(q, xi_ai, g_sli0, Mi, nDoF ,nD, vert_axis)
error_code = 0; % all is good

% [11-2-23] Updated to work for 3 dof no-smm robots

g_earth = -9.80665; % [m/s^2]
switch nD
    case '2D'
        g = zeros(2,1);
        switch vert_axis
            case 'x'
                g(1) = g_earth; 
            case 'y'
                g(2) = g_earth;
            case 'z'
                g(2) = g_earth;
        end
    case '3D'
        g = zeros(3,1);
        switch vert_axis
            case 'x'
                g(1) = g_earth; 
            case 'y'
                g(2) = g_earth;
            case 'z'
                g(3) = g_earth;
        end
    otherwise
        error_code = 1;
end

% Compute the FWD Kinematics for CoM origin frames
g_sli = zeros(4,4,nDoF);
if (nDoF == 2)
    g_sli(:,:,1) = twistexp(xi_ai(:,1),q(1)) * g_sli0(:,:,1);
    g_sli(:,:,2) = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * g_sli0(:,:,2);
elseif (nDoF == 3)
    g_sli(:,:,1) = twistexp(xi_ai(:,1),q(1)) * g_sli0(:,:,1);
    g_sli(:,:,2) = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * g_sli0(:,:,2);
    g_sli(:,:,3) = twistexp(xi_ai(:,1),q(1)) * twistexp(xi_ai(:,2),q(2)) * twistexp(xi_ai(:,3),q(3))* g_sli0(:,:,3);
else
    error_code = 2;
end
% Given eq.(7.35)/Siciliano
Ul = zeros(nDoF,1);
for i=1:nDoF
    ml(i) = Mi(1,1,i);
    Ul(i) = - ml(i) * g' * g_sli(1:nDoF,4,i);
end

end