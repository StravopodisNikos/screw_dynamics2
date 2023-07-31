function [M_419] = compute_M_419(Ji,Mi,n)

M = zeros(n,n);

for i = 1:n
    M = M + Ji(:,:,i)' * Mi(:,:,i) * Ji(:,:,i);
end

M_419 = M;

end