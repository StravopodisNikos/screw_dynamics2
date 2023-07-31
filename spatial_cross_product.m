function [adX] = spatial_cross_product(X)
% X must be a 6x1 twist vector

adX(1,:) = [ 0   -X(6)   X(5)    0   -X(3)   X(2)];
adX(2,:) = [ X(6)    0  -X(4)    X(3) 0     -X(1)];
adX(3,:) = [-X(5) X(4)   0      -X(2) X(1)   0   ];
adX(4,:) = [ 0    0      0       0   -X(6)   X(5)];
adX(5,:) = [ 0    0      0       X(6) 0     -X(4)];
adX(6,:) = [ 0    0      0      -X(5) X(4)   0   ];

end