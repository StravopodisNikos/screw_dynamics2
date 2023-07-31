function [dJb_ij_dt] = calculateFirstTimeDerivativeBodyJacobian2_Mueller(Jb_bodies,dq,Ci,XI_ii,nBodies,nDoF)
% Implements the SECOND (=) of eq.(8) in Mueller Dynamics
% Jb_ij, where i denotes the body of the manipulator the Body Jacobian
% relates to and j the current joint variable counter
% Jb_bodies = [Jb_1j | Jb_2j | ... | Jb_nj], for a serial chain of n bodies

i = nDoF; % assumes the Jacobian relates to the {T} frame after the last active twist

dJb_ij = sym(zeros(6,nDoF),'d');

for j=1:i
    dJ = sym(zeros(6,1),'d');
    for k=j+1:i
        % Adad1,2,3 correspond to each (=) in first part of eq.(8) in Mueller Dynamics Paper
        % The 3rd is used as this is the final one given by Mueller.
%         Adad1 = simplify( liebracket_426_new(Jb_bodies(:,j,nBodies), Jb_bodies(:,k,nBodies)) );
%         Adad2 = simplify( ad(inv(Ci(:,:,nBodies)) * Ci(:,:,k) ) * liebracket_426_new(Jb_bodies(:,j,k),XI_ii(:,k)) );
        Adad3 = simplify(- ad(inv(Ci(:,:,nBodies)) * Ci(:,:,k) ) * spatial_cross_product(XI_ii(:,k))* Jb_bodies(:,j,k) );
        dJ = dJ + (Adad3) * dq(k);
    end
    
    dJb_ij(:,j) = dJ;
end

dJb_ij_dt = simplify(dJb_ij);
end