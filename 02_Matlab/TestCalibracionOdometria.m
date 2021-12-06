%% Test de calibración de odometría (incremental)
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
format longG
global time_unit

load calibracion_odometria % A

%% Coloación del robot
N_medidas = 100;
N_sensores = 3;

apoloPlaceMRobot('Marvin',[-12 10 0],0,'World 1');
apoloUpdate();

%% Pruebas
v = 0.6;
w = 0.2;

odometria = zeros(N_medidas, 3);

var_odo_teor = Q_pu * [v; v*w; w].^2

for i = 1:N_medidas
    apoloResetOdometry('Marvin', [0 0 0],'World 1');

    apoloMoveMRobot('Marvin',[v w], time_unit, 'World 1');
    apoloUpdate();
    
    odometria_next = apoloGetOdometry('Marvin','World 1');

    odometria(i, :) = odometria_next;
    
    
end

var_odo = var(odometria)

%% Resultados
figure(10);
plot(odometria(:, 1));
title("medidas de la odometría " + num2str(1));



