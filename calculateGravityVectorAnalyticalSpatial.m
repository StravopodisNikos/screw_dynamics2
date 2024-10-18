function [g_sp] = calculateGravityVectorAnalyticalSpatial(q, xi_ai, Jsg, mi, gsli0, gst0)
g_earth = -9.80665; % [m/s^2]
g = [0 0 g_earth]';
Fgi_s(:,1) = mi(1) * g;
Fgi_s(:,2) = mi(2) * g;
Fgi_s(:,3) = mi(3) * g;

for i=1:3
    exp_ai(:,:,i) = twistexp( xi_ai(:,i),q(i) );
end
gsli(:,:,1) = exp_ai(:,:,1) * gsli0(:,:,1);
gsli(:,:,2) = exp_ai(:,:,1) * exp_ai(:,:,2) *gsli0(:,:,2);
gsli(:,:,3) = exp_ai(:,:,1) * exp_ai(:,:,2) * exp_ai(:,:,3) *gsli0(:,:,3);
gst = exp_ai(:,:,1) * exp_ai(:,:,2) * exp_ai(:,:,3) *gst0;

tau_g_1_s = Jsg(1:3,:,1)' * Fgi_s(:,1);
tau_g_2_s = Jsg(1:3,:,2)' * Fgi_s(:,2);
tau_g_3_s = Jsg(1:3,:,3)' * Fgi_s(:,3);
g_sp = tau_g_1_s + tau_g_2_s + tau_g_3_s;
end