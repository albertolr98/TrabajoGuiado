%% Calibración de sensores
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
clear
format shortG

global robot_name

variables_globales

%% Toma de datos
N_medidas = 500;
N_sensores = 3;

apoloPlaceMRobot(robot_name,[4 2 0],-pi/2);
apoloUpdate();

%% Ultrasonidos (sin moverse)
medidas = zeros(N_medidas, N_sensores);

for i = 1:N_medidas
    medidas_next = GetUltrasonicSensorsWithNoise(robot_name);

    medidas(i, :) = medidas_next;
end

%% Dibujos
figure;
for i = 1:N_sensores
    hold on
    plot(1:N_medidas, medidas(:, i));
    xlabel('z (m)');
    ylabel('medida');
end

var_med = var(medidas);

R = diag(var_med)

save calibracion_sensores R