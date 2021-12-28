function Z = GetUltrasonicSensorsWithNoise(robot_name)
%Z=GETULTRASONICSENSORSWITHNOISE(robot_name)
% Obtiene las medidas de los ultrasonidos (apoloGetAllultrasonicSensors) y
% les suma un ruido gaussiano para simular sus errores en la medida.

normal_distr = makedist('Normal', 'mu', 0, 'sigma', 1e-3);

Z_real = apoloGetAllultrasonicSensors(robot_name)';
Z = Z_real + normal_distr.random(size(Z_real));

end

