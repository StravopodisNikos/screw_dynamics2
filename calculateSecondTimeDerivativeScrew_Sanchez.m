function [ddSi_dt] = calculateSecondTimeDerivativeScrew_Sanchez(i,Js,dq,ddq)
% Implements eq.(14) in paper:
% "The differential calculus of screws: theory, geometrical interpretation
% and applications", Sanchez 2008

% CAUTION: DOESNT SEEM GOOD

dS1 = sym(zeros(6,1),'d');
dS2 = sym(zeros(6,1),'d');
dS3 = sym(zeros(6,1),'d');

for j=1:i-1
    dS1 = dS1 + ddq(j) * liebracket_426_new2(Js(:,j), Js(:,i));
end

for j=1:i-1
    dS2 = dS2 + dq(j) *dq(j) * liebracket_426_new2(Js(:,j), liebracket_426_new2(Js(:,j),Js(:,i)) );
end

for j=1:i-2
    
    dS4 = sym(zeros(6,1),'d');
    for k=j+1:i-1
        dS4 = dS4 + dq(k) * liebracket_426_new2(Js(:,j), liebracket_426_new2(Js(:,k),Js(:,i)) );
    end
    dS3 = dS3 + 2 *dq(j) * dS4;
end

ddS_i = dS1 + dS2 + dS3;

ddSi_dt = ddS_i;
end