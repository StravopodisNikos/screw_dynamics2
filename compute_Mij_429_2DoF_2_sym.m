function [Mij_429,delta_Mij_thetak_429,Cij_429,Ck,Gamma_ijk] = compute_Mij_429_2DoF_2_sym(xi_ai, exp_ai, Pi, gsli0, Mi_b, theta_dot)
% Computes Manipulator Inertia Matrix based on eq.4.29a p.176
% [29-4-21] Computes Coriolis Matric,Christoffel symbols and Ck(q) defined in Loria

n_Dof = size(xi_ai,2); % extract number of links
Mij_429 = sym(zeros(n_Dof),'r');
Cij_429 = sym(zeros(n_Dof),'r');
dM2C = sym(zeros(n_Dof),'r');

i = n_Dof;
j = n_Dof;
for c_i=1:i
    for c_j=1:j
        % Compute Mij
        l = max(c_i,c_j);
        for add=l:n_Dof
            [Aij_1] = compute_Aij_for_Ji_427_2DoF_sym(exp_ai, Pi, add, c_i);
            [Aij_2] = compute_Aij_for_Ji_427_2DoF_sym(exp_ai, Pi, add, c_j);
            
            Ml = ad(inv(gsli0(:,:,add)))'*Mi_b(:,:,add)*ad(inv(gsli0(:,:,add)));
            Mij_429(c_i,c_j) = Mij_429(c_i,c_j) + xi_ai(:,c_i)'*Aij_1'*Ml*Aij_2*xi_ai(:,c_j);
        end
        
        % Compute  δ(Mij)/δ(θk) for all θk for each Mij
        Cij_429(c_i,c_j) = sym(0,'r');
        for k=1:n_Dof
            % Mass matrix partial derivatives given in eq.4.29,p176 Murray
            [delta_Mij_thetak_429(c_i,c_j,k)] = compute_delta_Mij_thetak_429_2DoF_2_sym(xi_ai, exp_ai, Pi, gsli0, Mi_b, c_i,  c_j,   k);
            [delta_Mik_thetaj_429(c_i,c_j,k)] = compute_delta_Mij_thetak_429_2DoF_2_sym(xi_ai, exp_ai, Pi, gsli0, Mi_b, c_i,  k  , c_j);
            [delta_Mkj_thetai_429(c_i,c_j,k)] = compute_delta_Mij_thetak_429_2DoF_2_sym(xi_ai, exp_ai, Pi, gsli0, Mi_b,   k,  c_j, c_i);
            
            % Christoffel symbol Γijk
            Gamma_ijk(c_i,c_j,k) = 0.5 * ( delta_Mij_thetak_429(c_i,c_j,k) + delta_Mik_thetaj_429(c_i,c_j,k) - delta_Mkj_thetai_429(c_i,c_j,k) );
            
            % Coriolis element Cij
            Cij_429(c_i,c_j) = Cij_429(c_i,c_j) + ( Gamma_ijk(c_i,c_j,k) * theta_dot(k) ); 
        end    
        
    end
end

%% Coriolis matrix given eq.3.22,p.73 Loria
% Cij_429_loria(1,1) = [Gamma_ijk(1,1,1); Gamma_ijk(2,1,1); Gamma_ijk(3,1,1)]' * theta_dot; %
% Cij_429_loria(1,2) = [Gamma_ijk(1,2,1); Gamma_ijk(2,2,1); Gamma_ijk(3,2,1)]' * theta_dot;
% Cij_429_loria(1,3) = [Gamma_ijk(1,3,1); Gamma_ijk(2,3,1); Gamma_ijk(3,3,1)]' * theta_dot;
% Cij_429_loria(2,1) = [Gamma_ijk(1,1,2); Gamma_ijk(2,1,2); Gamma_ijk(3,1,2)]' * theta_dot; %
% Cij_429_loria(2,2) = [Gamma_ijk(1,2,2); Gamma_ijk(2,2,2); Gamma_ijk(3,2,2)]' * theta_dot;
% Cij_429_loria(2,3) = [Gamma_ijk(1,3,2); Gamma_ijk(2,3,2); Gamma_ijk(3,3,2)]' * theta_dot;
% Cij_429_loria(3,1) = [Gamma_ijk(1,1,3); Gamma_ijk(2,1,3); Gamma_ijk(3,1,3)]' * theta_dot; %
% Cij_429_loria(3,2) = [Gamma_ijk(1,2,3); Gamma_ijk(2,2,3); Gamma_ijk(3,2,3)]' * theta_dot;
% Cij_429_loria(3,3) = [Gamma_ijk(1,3,3); Gamma_ijk(2,3,3); Gamma_ijk(3,3,3)]' * theta_dot;

%% Check for skew symmetric Mdot-2C -> [28-4-21] Lemma 4.2,p.171 Murray!
% and construct the n Ck matrices defined in Property 4.2[4],p.97 Loria!
for c_i=1:i
    for c_j=1:j
        for k=1:n_Dof
            % Mdot - 2 * C 
            dM2C(c_i,c_j) = dM2C(c_i,c_j) + ( delta_Mij_thetak_429(c_i,k,c_j) * theta_dot(k) )  - ( delta_Mij_thetak_429(k,c_j,c_i) * theta_dot(k) );
        
            % Ck of property 4.2[4] in Loria -> n Ck matrices that MUST be symmetric nxn matrices! 
             Ck(k,c_i,c_j) = Gamma_ijk(c_j,c_i,k);
        end
    end
end
% check = isskew(dM2C);

end