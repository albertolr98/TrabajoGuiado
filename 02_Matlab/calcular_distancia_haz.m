function [z, X_m] = calcular_distancia_haz(X, p)
    %[z, X_m] = CALCULAR_DISTANCIA_HAZ([x y theta], pared)
    %   Calcula la distancia del punto [x y] a la pared, según un
    %   haz con un ángulo theta. Devuelve también las coordenadas de ese
    %   punto X_m, para poder ponerlo en una imagen, o donde sea.
    x = X(1);
    y = X(2);
    theta = X(3);

    if p.tipo == 'h' % pared horizontal
        y_p = p.X1(2);

        % X corte con la pared
        x_p = x + 1/tan(theta)*(y_p - y);

        if ~esta_en_pared(p, [x_p; y_p])
            z = inf;
        elseif sign(y_p - y) ~= sign(sin(theta)) % comprobar cuadrante correcto
            z = inf;
        else
            z = (y_p - y)/sin(theta);
        end
    elseif p.tipo == 'v' % pared vertical
        x_p = p.X1(1);

        % Y corte con la pared
        y_p = y + tan(theta)*(x_p - x);

        if ~esta_en_pared(p, [x_p; y_p])
            z = inf;
        elseif sign(x_p - x) ~= sign(cos(theta)) % comprobar cuadrante correcto
            z = inf;
        else
            z = (x_p - x)/cos(theta);
        end
    end

    X_m = [x_p y_p];
end 