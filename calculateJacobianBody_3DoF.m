function [Jb] = calculateJacobianBody_3DoF(q, xi_ai, gst0)

    nDoF = size(xi_ai,2);
    for i=1:nDoF
        exp_ai(:,:,i) = twistexp( xi_ai(:,i),q(i) );
    end
    
    Jb = zeros(6,nDoF);
    
    Jb(:,1) = ad(inv(exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,1);
    Jb(:,2) = ad(inv(exp_ai(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai(:,2);
    Jb(:,3) = ad(inv(exp_ai(:,:,3)*gst0))*xi_ai(:,3);
    
end