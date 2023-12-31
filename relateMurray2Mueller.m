function [s_mueller] = relateMurray2Mueller(s_murray,nDoF)
% [8-2-23] Relates the notation between the 2 papers.
% Implemented for code clarity, theory revision and debugging.

% Notations Correlation Table
%-----------------------------------------
% Murray        --->    Mueller
%-----------------------------------------
% gsa_i(0)    |     |   Ai = Ci(0)
% gsa_i_i-1(0)|     |   Bi   = C_i_i-1(0)
% xi_ai       |     |   YI_i
% xi_ai_i     |     |   XI_ii
%             |     |   XI_i_i1
%-----------------------------------------
% Following are implemented in:
% calculateFwdKinMueller.m
% gsa_i(q)    |     |   Ci
% gsa_i_i-1(q)|     |   Ci_i-1

%% Ai = Ci(0) = gsai(0)
for i=1:nDoF
    Ai(:,:,i) = s_murray.gsai0(:,:,i);
end
Ai(:,:,nDoF+1)= s_murray.gst0;
field1 = 'Ai'; value1 = Ai; 
%% Bi = Ci_i1(0)
Bi(:,:,1) = s_murray.gsai0(:,:,1); % g_s_1_0
for i=2:(nDoF+1)
  Bi(:,:,i) = inv(Ai(:,:,i-1)) * Ai(:,:,i);% g_i_i-1(0)  
end
field2 = 'Bi'; value2 = Bi;
%% YI_i
for i=1:nDoF
    YI_i(:,i) = s_murray.xi_ai(:,i);
%     YI_i(1:3,i) = s_murray.xi_ai(4:6,i);
%     YI_i(4:6,i) = s_murray.xi_ai(1:3,i);
end
field3 = 'YI_i'; value3 = YI_i;
%% XI_ii
for i=1:nDoF
    XI_ii(:,i) = ad(inv(Ai(:,:,i)))*YI_i(:,i);
end
XI_ii(:,nDoF+1) = [Bi(1:3,4,nDoF+1); vee(Bi(1:3,1:3,nDoF+1))];
field4 = 'XI_ii'; value4 = XI_ii;
%% XI_i_i1 
for i=1:nDoF+1
    XI_i_i1(:,:,i) = ad(Bi(:,:,i)) * XI_ii(:,i);
end
field5 = 'XI_i_i1'; value5 = XI_i_i1;

s_mueller = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5);
end