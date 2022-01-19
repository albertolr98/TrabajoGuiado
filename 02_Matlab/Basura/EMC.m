function [X_0, P_0] = EMC (robot, entorno, it)
global robot_name laser_name nbalizas

load('calibracion_sensores', 'R');
R_acumulado = [];
H_acumulado = [];
Z_acumulado = [];

% Esto es solo necesario para sacar la matriz H
[~, Hk, ~] = robot.estimar_medidas(entorno);

for j = 1:it
    %% Medida de los ultrasonidos - se pasa de ellos
    Z1_k = zeros(5, 1);
    
    %% Medida de las balizas
    if isa(robot.sensores(end),'sensor_ls')
        Z2_k = GetLaserData(laser_name, nbalizas);
    end
    
    %% Medida de todos los sensores
    if isa(robot.sensores(end),'sensor_ls')
        Z_k = [Z1_k; Z2_k];
    else
        Z_k = Z1_k;
    end

    % Eliminación de medidas de ultrasonidos fuera de su rango de aplicación
    for i = 1:length(Z1_k) 
        if Z_k(i) > 2.9
            Z_k(i) = 0;
        end
    end
    
    % Eliminación de medidas de láser cuando no se ven las balizas
    for i = length(Z1_k)+1:length(Z_k) 
        if isnan(Z_k(i))
            Z_k(i) = 0;
        end
    end

    %% Matrices acumuladas
    R_acumulado = blkdiag(R_acumulado, R); %#ok<*AGROW> 
    H_acumulado = [H_acumulado; Hk];
    Z_acumulado = [Z_acumulado; Z_k];
end


P_0 = inv(H_acumulado'/R_acumulado*H_acumulado);
X_0 = P_0*H_acumulado'/R_acumulado*Z_acumulado;

end


