%% Calibración de sensores
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
format shortG

N_medidas = 100;
N_sensores = 3;

apoloPlaceMRobot('Marvin',[4 2 0],-pi/2,'World 1');
apoloUpdate();

%% Ultrasonidos (sin moverse)
medidas = zeros(N_medidas, N_sensores);

for i = 1:N_medidas
    medidas_next = apoloGetAllultrasonicSensors('Marvin','World 1');

    medidas(i, :) = medidas_next;
end

var_med = var(medidas);

% figuras
% for i = 1:N_sensores
%     figure(i);
%     plot(medidas(:, i));
%     title("medidas del sensor " + num2str(i));
% end