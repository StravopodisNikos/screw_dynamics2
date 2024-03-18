function [Jsgl] = calculateLinkSpatialJacobians_3DoF(q, xi_ai)
Jsgl = zeros(6,3,3);
O61 = zeros(6,1);

for i=1:3
    exp_ai(:,:,i) = twistexp( xi_ai(:,i),q(i) );
end

Jsgl_1_1 = xi_ai(:,1);
Jsgl(:,:,1) = [Jsgl_1_1 O61 O61];

Jsgl_1_2 = xi_ai(:,1);
Jsgl_2_2 = ad( exp_ai(:,:,1)) * xi_ai(:,2);
Jsgl(:,:,2) = [Jsgl_1_2 Jsgl_2_2 O61];

Jsgl_1_3 = xi_ai(:,1);
Jsgl_2_3 = ad( exp_ai(:,:,1)) * xi_ai(:,2);
Jsgl_3_3 = ad( exp_ai(:,:,1) * exp_ai(:,:,2)) * xi_ai(:,3);
Jsgl(:,:,3) = [Jsgl_1_3 Jsgl_2_3 Jsgl_3_3];

end