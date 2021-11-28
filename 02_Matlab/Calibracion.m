%% Calibración de sensores y odometría
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

%% Odometría (moviéndose)
odometria = zeros(N_medidas, 3);

for i = 1:N_medidas
    apoloResetOdometry('Marvin', [0 0 0],'World 1');

    apoloMoveMRobot('Marvin',[-0.2 0.1], 0.05, 'World 1');
    apoloUpdate();
    
    odometria_next = apoloGetOdometry('Marvin','World 1');

    odometria(i, :) = odometria_next;
    
%     apoloResetOdometry(robot,pose,world)
end

var_odo = var(odometria)

% figuras
for i = 1:3
    figure(100 + i);
    plot(odometria(:, i));
    title("medidas de la odometría " + num2str(i));
end