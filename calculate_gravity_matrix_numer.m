function [N,V] = calculate_gravity_matrix_numer(dof_string, mi, gsli, Vold, delta_q)
% FUNCTION: DISCONTNUED!!!! -> NEW FN calculate_gravity_matrix_numer_anat
% Numerically calculates the gradient of Gravity matrix @ each C-space
% point. gsli is comuted at each C-space point and passed with delta_q vector to
% this function

switch dof_string
    case '3dof'
        n_Dof = 3;
    case '6dof'
        n_Dof = 6;
end
 
g = -9.8067; % [m/s^2]

V = zeros(n_Dof,1);
DeltaV = zeros(n_Dof,1);
N = zeros(n_Dof,1);

for i = 1:n_Dof
    V(i) = mi(i)*g*gsli(3,4,i);
    DeltaV(i) = (V(i) - Vold(i) );
    if (delta_q(i) == 0)
        delta_q(i) = 0.001;
    end
    N(i) = DeltaV(i) / delta_q(i);

end

end