function [Ci,Ci_i1] = calculateFwdKinMueller(s_mueller,q,nDoF)
% [8-2-23]
% s_mueller is constructed by relateMurray2Mueller.m

for i=1:nDoF
    Ci_i1(:,:,i) = s_mueller.Bi(:,:,i) * twistexp(s_mueller.XI_ii(:,1),q(i));
end
 Ci_i1(:,:,nDoF+1) = s_mueller.Bi(:,:,nDoF+1);
 
switch nDoF
    case 1
        Ci(:,:,1) = Ci_i1(:,:,1);
        Ci(:,:,2) = Ci_i1(:,:,1)*Ci_i1(:,:,2); % gst
    case 2
        Ci(:,:,1) = Ci_i1(:,:,1);
        Ci(:,:,2) = Ci_i1(:,:,1)*Ci_i1(:,:,2);
        Ci(:,:,3) = Ci_i1(:,:,1)*Ci_i1(:,:,2)*Ci_i1(:,:,3);
    case 3
        Ci(:,:,1) = Ci_i1(:,:,1);
        Ci(:,:,2) = Ci_i1(:,:,1)*Ci_i1(:,:,2);
        Ci(:,:,3) = Ci_i1(:,:,1)*Ci_i1(:,:,2)*Ci_i1(:,:,3);
        Ci(:,:,4) = Ci_i1(:,:,1)*Ci_i1(:,:,2)*Ci_i1(:,:,3)*Ci_i1(:,:,4);
    case 4
        Ci(:,:,1) = Ci_i1(:,:,1);
        Ci(:,:,2) = Ci_i1(:,:,1)*Ci_i1(:,:,2);
        Ci(:,:,3) = Ci_i1(:,:,1)*Ci_i1(:,:,2)*Ci_i1(:,:,3);
        Ci(:,:,4) = Ci_i1(:,:,1)*Ci_i1(:,:,2)*Ci_i1(:,:,3)*Ci_i1(:,:,4);
        Ci(:,:,5) = Ci_i1(:,:,1)*Ci_i1(:,:,2)*Ci_i1(:,:,3)*Ci_i1(:,:,4)*Ci_i1(:,:,5);
    otherwise
        warning('[calculateFwdKinMueller]: nDoF MUST be lower/equal to 4')
end

end