function [n_num,n,U,error_code] = calculate_gravity_vector(qnum, xi_ai, gsli0, Mi, nDoF ,nD, vert_axis)

% Create a symbolic vector for joint position variables
q = sym('q',[nDoF 1]);

% Calculate the Potential Energy 
[U,error_code] = calculate_potential_energy(q, xi_ai, gsli0, Mi, nDoF ,nD, vert_axis);
U = simplify(U);

% Calculate the gravity vector
n = simplify(gradient(U,q));

% Assign numeric values 
switch nDoF
    case 3
        n(1) = subs(n(1),[q(1) q(2) q(3)], [qnum(1) qnum(2) qnum(3)]);
        n(2) = subs(n(2),[q(1) q(2) q(3)], [qnum(1) qnum(2) qnum(3)]);
        n(3) = subs(n(3),[q(1) q(2) q(3)], [qnum(1) qnum(2) qnum(3)]);
    otherwise
        warning('[calculate_gravity_vector]: Not supported nDOF.')
end

n_num = double(n);
end