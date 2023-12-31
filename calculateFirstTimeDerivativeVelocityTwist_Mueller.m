function [dV_i_s_dt1, dV_i_s_dt2, dV_i_b1] = calculateFirstTimeDerivativeVelocityTwist_Mueller(Js,dJs,Jb,dJb,ddq,dq, USE_SYM)
% Implements eq.(18) in Mueller Dynamics, for spatial
% dV_i_s_dt1 is the 1st "="
% dV_i_s_dt2 is the 2nd "="
% Implements eq.(7) in Mueller Dynamics, for body
% dV_i_b_dt1 is the 1st "="

nDoF = size(Js,2);

if (USE_SYM)
    dV_i_s1 = sym(zeros(6,1),'d');
    dV_i_s2 = sym(zeros(6,1),'d');
    dV_i_b1 = sym(zeros(6,1),'d');
else
    dV_i_s1 = zeros(6,1);
    dV_i_s2 = zeros(6,1);
    dV_i_b1 = zeros(6,1);
end

%% Spatial Velocity twists
%% 1st "="
for i=1:nDoF
    if (USE_SYM)
        dV1 = sym(zeros(6,1),'d');
        dV2 = sym(zeros(6,1),'d');
    else
        dV1 = zeros(6,1);
        dV2 = zeros(6,1);
    end

    for j=1:i
    dV1 = dV1 + Js(:,j) * ddq(j);
    end
        
    for k=1:i
        for j=k+1:i
            dV2 = dV2 + liebracket_426_new(Js(:,k), Js(:,j)) * dq(j) * dq(k) ;
        end
    end
    
    dV_i_s1(:,i) = dV1 + dV2;
end

dV_i_s_dt1 = dV_i_s1;

%% 2nd "="
for i=1:nDoF
    if (USE_SYM)
        dV = sym(zeros(6,1),'d');
    else
        dV = zeros(6,1);
    end
    for j=1:i
        dV = dV + ( Js(:,j) * ddq(j) + dJs(:,j) * dq(j));
    end
    
    dV_i_s2(:,i) = dV;
end

dV_i_s_dt2 = dV_i_s2;

% Verified in [20-9-23], Both return the same result

%% Body Velocity twists
% for i=4(the {T} frame)
dV_i_b1 = Jb(:,:,4) * ddq + dJb * dq;
end