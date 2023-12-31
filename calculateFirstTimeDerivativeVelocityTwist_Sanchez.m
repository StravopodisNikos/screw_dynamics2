function [dVs_dt] = calculateFirstTimeDerivativeVelocityTwist_Sanchez(Js,dS,ddq,dq)
% Implements eq.(28) in paper:
% "The differential calculus of screws: theory, geometrical interpretation
% and applications", Sanchez 2008
% Js -> its columns are the screws S of Sanchez
% dS -> are the first derivatives of the S screws (calculateFirstTimeDerivativeScrew_Sanchez MUST be previously executed)

S1 = Js(:,1);
S2 = Js(:,2);
S3 = Js(:,3);
dS1 = dS(:,1);
dS2 = dS(:,2);
dS3 = dS(:,3);

dVs_dt(:,1) = S1*ddq(1) +  dS1 * dq(1);
dVs_dt(:,2) = S1*ddq(1) + S2*ddq(2) + dS1 * dq(1) + dS2 * dq(2);
dVs_dt(:,3) = S1*ddq(1) + S2*ddq(2) + S3*ddq(3) + dS1 * dq(1) + dS2 * dq(2) + dS3 * dq(3);

end
