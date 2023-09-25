function [Ci_1, Ci_2 ,Ci_i1] = calculateFwdKinMueller(s_mueller,q,nDoF)
% [8-2-23]
% s_mueller is constructed by relateMurray2Mueller.m

% for i=1:nDoF
%     Ci_i1(:,:,i) = twistexp(s_mueller.XI_i_i1(:,1),q(i)) * s_mueller.Bi(:,:,i) ;
% end
Ci_i1 = 0; % dummy for now...

switch nDoF
    case 1
        Ci_1(:,:,1) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1));
        Ci_2(:,:,1) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1);
    case 2
        Ci_1(:,:,1) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1));
        Ci_2(:,:,1) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1);
        Ci_1(:,:,2) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1)) * s_mueller.Bi(:,:,2) * twistexp(s_mueller.XI_ii(:,2),q(2));
        Ci_2(:,:,2) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_i_i1(:,2),q(2)) * s_mueller.Bi(:,:,2);
    case 3
        % for joint axes
        Ci_1(:,:,1) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1));
        Ci_2(:,:,1) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1);
        Ci_1(:,:,2) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1)) * s_mueller.Bi(:,:,2) * twistexp(s_mueller.XI_ii(:,2),q(2));
        Ci_2(:,:,2) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_i_i1(:,2),q(2)) * s_mueller.Bi(:,:,2);        
        Ci_1(:,:,3) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1)) * s_mueller.Bi(:,:,2) * twistexp(s_mueller.XI_ii(:,2),q(2)) * s_mueller.Bi(:,:,3) * twistexp(s_mueller.XI_ii(:,3),q(3));
        Ci_2(:,:,3) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_i_i1(:,2),q(2)) * s_mueller.Bi(:,:,2) * twistexp(s_mueller.XI_i_i1(:,3),q(3)) * s_mueller.Bi(:,:,3);
        % for tool frame
        Ci_1(:,:,4) = s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_ii(:,1),q(1)) * s_mueller.Bi(:,:,2) * twistexp(s_mueller.XI_ii(:,2),q(2)) * s_mueller.Bi(:,:,3) * twistexp(s_mueller.XI_ii(:,3),q(3)) * s_mueller.Bi(:,:,4);
        Ci_2(:,:,4) = twistexp(s_mueller.XI_i_i1(:,1),q(1)) * s_mueller.Bi(:,:,1) * twistexp(s_mueller.XI_i_i1(:,2),q(2)) * s_mueller.Bi(:,:,2) * twistexp(s_mueller.XI_i_i1(:,3),q(3)) * s_mueller.Bi(:,:,3) * s_mueller.Bi(:,:,4);  
    otherwise
        warning('[calculateFwdKinMueller]: nDoF MUST be lower/equal to 3')
end

end