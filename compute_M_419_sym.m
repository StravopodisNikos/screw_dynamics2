function [M_419] = compute_M_419_sym(Ji,Mi,n)

M = sym(zeros(n,n),'r');

for i = 1:n
    M = M + Ji(:,:,i)' * Mi(:,:,i) * Ji(:,:,i);
end

M_419 = M;

end