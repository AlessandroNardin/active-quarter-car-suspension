% Jacobian of h with respect to v
function Mk = suspension_M()
% additive measurement noise => identity
Mk = eye(3);
end