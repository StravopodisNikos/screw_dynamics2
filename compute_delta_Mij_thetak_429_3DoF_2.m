function [delta_Mij_thetak_429] = compute_delta_Mij_thetak_429_3DoF_2(xi_ai, exp_ai, Pi, gsli0, Mi_b, i, j, k)
% Computes Manipulator Inertia Matrix based on eq.4.29a p.176
% [11-4-21] Developed to work with compute_Mij_429_3DoF_2

% l = max(i,j);
n_Dof = size(xi_ai,2); % extract number of links
% delta_Mij_thetak_429 = zeros(n_Dof);
delta_Mij_thetak_429 = 0;

% i = n_Dof;
% j = n_Dof;
% for c_i=1:i
%     for c_j=1:j
c_i = i;
c_j = j;
        l = max(c_i,c_j);
        for add=l:n_Dof
            [Aki] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, k, c_i);
            [Alk] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, add, k);
            [Alj] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, add, c_j);
            Lie_Br_1 = liebracket_426_new(Aki*xi_ai(:,c_i), xi_ai(:,k));
            
            [Ali] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, add, c_i);
            [Akj] = compute_Aij_for_Ji_427_3DoF(exp_ai, Pi, k, c_j);
            Lie_Br_2 = liebracket_426_new(Akj*xi_ai(:,c_j), xi_ai(:,k));
            
            % [13-2-24] BUG found! Tf to {S} is executed during assembly
%              Ml = ad(inv(gsli0(:,:,add)))'*Mi_b(:,:,add)*ad(inv(gsli0(:,:,add)));
             Ml = Mi_b(:,:,add);
             
            delta_Mij_thetak_429 = delta_Mij_thetak_429 + (Lie_Br_1'*Alk'*Ml*Alj*xi_ai(:,c_j) + xi_ai(:,c_i)'*Ali'*Ml*Alk*Lie_Br_2);
        end
%         
%     end
% end

end