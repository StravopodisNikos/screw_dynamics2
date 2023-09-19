function [SP1] = screwProduct_symGeneration()
% Generates the symbolic analytical form of eq.115 in Mueller-Dynamics Paper
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_dynamics')

% First '='
xi_1 = sym('xi1_',[3 1], 'real');
htta_1 = sym('ht1_',[3 1], 'real');
XI1 = [xi_1; htta_1];

xi_2 = sym('xi2_',[3 1], 'real');
htta_2 = sym('ht2_',[3 1], 'real');
XI2 = [xi_2; htta_2];

SP1(1:3,1) = cross(xi_1, xi_2)
SP1(4:6,1) = cross(htta_1, xi_2) + cross(xi_1,htta_2);

% Second '='
adXI1(1:3,1:3) = skew(xi_1);
adXI1(1:3,4:6) = zeros(3);
adXI1(4:6,1:3) = skew(htta_1);
adXI1(4:6,4:6) = skew(xi_1);
SP2 = adXI1 * XI2

SP3 = liebracket_426_new(XI1, XI2)

XI1_1 = [htta_1; xi_1];
XI2_1 = [htta_2; xi_2];
SP3_1 = liebracket_426_new2(XI1_1, XI2_1)

end