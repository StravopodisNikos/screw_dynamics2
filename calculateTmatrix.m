function [T] = calculateTmatrix(C,A)
% Computes the T matrix multiplication used for the
% Body Manipulator Jacobian calculation based on Mueller KInematics Paper

T = C / A;

end