function [Aij_427] = compute_Aij_for_Ji_427_2DoF(exp_ai, Pi, i, j)

Aij_427 = zeros(6);

if i==1
    if j==1 % i=j
        Aij_427 = eye(6);
    elseif j==2 % i<j
        Aij_427 = zeros(6);    
    end
elseif i==2
    if j==1 % j<i
        Aij_427 = inv(ad(Pi*exp_ai(:,:,i)));
    elseif j==2 % i=j
        Aij_427 = eye(6);
    end
else
    
end

end