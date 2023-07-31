function [Aij_427] = compute_Aij_for_Ji_427_2DoF_sym(exp_ai, Pi, i, j)

Aij_427 = sym(zeros(6),'r');

if i==1
    if j==1 % i=j
        Aij_427 = sym(eye(6),'r');
    elseif j==2 % i<j
        Aij_427 = sym(zeros(6),'r');    
    end
elseif i==2
    if j==1 % j<i
        Aij_427 = inv(ad(Pi*exp_ai(:,:,i)));
    elseif j==2 % i=j
        Aij_427 = sym(eye(6),'r');
    end
else
    
end

end