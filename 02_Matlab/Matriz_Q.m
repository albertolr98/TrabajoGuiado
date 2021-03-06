function Q = Matriz_Q(velocidad, Q_pu)
%Q = MATRIZ_Q([v w]);
% Calcula la matriz de varianzas y covarianzas de la odometría Q en función
% de la velocidad v en m/s y de la velocidad de giro w en rad/s

v = velocidad(1);
w = velocidad(2);

variables = diag([v; v*w; w].^2); % variables para calcular Q
variables(1,2) = v^2*abs(w);
variables(2,1) = v^2*abs(w);
Q = Q_pu .* variables;

end