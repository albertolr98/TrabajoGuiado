function out = punto_en_cuadrante(X, beta)
%out = PUNTO_EN_CUADRANTE([x y], beta)
%   Comprueba que el punto [x y] está en el cuadrante correcto de beta.
beta = mod(beta, 2*pi);

x = X(1);
y = Y(1);

if beta < pi/2 % cuadrante 1
    if x < 0 || y < 0
        out = 0;
    else
        out = 1;
    end
elseif beta < pi % cuadrante 2
    if x > 0 || y < 0
        out = 0;
    else
        out = 1;
    end
elseif beta < 3*pi/2 % cuadrante 3
    if x > 0 || y > 0
        out = 0;
    else
        out = 1;
    end
else % cuadrante 4
    if x < 0 || y > 0
        out = 0;
    else
        out = 1;
    end
end
    


end

