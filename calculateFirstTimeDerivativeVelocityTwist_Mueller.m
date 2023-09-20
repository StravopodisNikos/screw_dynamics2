function [dV_i_s_dt1, dV_i_s_dt2] = calculateFirstTimeDerivativeVelocityTwist_Mueller(Js,dJs,ddq,dq)
% Implements eq.(18) in Mueller Dynamics
% dV_i_s_dt1 is the 1st "="
% dV_i_s_dt2 is the 2nd "="

nDoF = size(Js,2);
%% 1st "="
dV_i_s1 = sym(zeros(6,1),'d');

for i=1:nDoF
    dV1 = sym(zeros(6,1),'d');
    dV2 = sym(zeros(6,1),'d');
    for j=1:i
    dV1 = dV1 + Js(:,j) * ddq(j);
    end
        
    for k=1:i
        for j=k+1:i
            dV2 = dV2 + liebracket_426_new2(Js(:,k), Js(:,j)) * dq(j) * dq(k) ;
        end
    end
    
    dV_i_s1(:,i) = dV1 + dV2;
end

dV_i_s_dt1 = simplify(dV_i_s1);

%% 2nd "="
dV_i_s2 = sym(zeros(6,1),'d');

for i=1:nDoF
    dV = sym(zeros(6,1),'d');
    
    for j=1:i
        dV = dV + ( Js(:,j) * ddq(j) + dJs(:,j) * dq(j));
    end
    
    dV_i_s2(:,i) = dV;
end

dV_i_s_dt2 = simplify(dV_i_s2);

% Verified in [20-9-23], Both return the same result
end