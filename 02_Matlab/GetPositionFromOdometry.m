function [X_k1_k, P_k1_k] = GetPositionFromOdometry(X_k_k, P_k_k, Q)
%[X(k+1|k), P(k+1|k)] = GETPOSITIONFROMODOMETRY(X(k|k), P(k|k), Q(k))
% Calcula la nueva posición del robot a partir de la odometría y los
% valores anteriores, así como la matriz de varianzas P asociada.
odometria = apoloGetOdometry('Marvin','World 1')';

theta = X_k_k(3);
M = [cos(theta) -sin(theta)  0;
     sin(theta) cos(theta)   0;
     0          0            1]; % matriz de rotación

% cálculo de X estimado
X_k1_k = X_k_k + M*odometria;
X_k1_k(3) = wrapToPi(X_k1_k(3));

% Jacobiano respecto a X estimado
F_x = [1 0 (-sin(theta)*odometria(1) -cos(theta)*odometria(2));
       0 1 (cos(theta)*odometria(1) -sin(theta)*odometria(2)) ;
       0 0 1];
   
% Jacobiano respecto a la odometría
F_u = M;

% Matriz P nueva
P_k1_k = F_x'*P_k_k*F_x + F_u'*Q*F_u;


end