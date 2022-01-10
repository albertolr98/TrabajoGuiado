%% Calibración de sensores
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
clear
format shortG

global robot_name laser_name nbalizas

variables_globales

%% Toma de datos
N_medidas = 500;
N_sensores_us = 5;

apoloPlaceMRobot(robot_name,[5 2 0],0);
apoloUpdate();

%% Ultrasonidos (sin moverse)
medidas_us = zeros(N_medidas, N_sensores_us);
medidas_ls = zeros(N_medidas, 1); % solo se mide el ángulo del sensor nº 1

for i = 1:N_medidas
    medidas_next_us = GetUltrasonicSensorsWithNoise(robot_name);
    medidas_next_ls = GetLaserData(laser_name, nbalizas);

    medidas_us(i, :) = medidas_next_us;
    medidas_ls(i, :) = medidas_next_ls(1);

end

var_med_us = var(medidas_us);
var_med_ls = var(medidas_ls);

R = diag([var_med_us repmat(var_med_ls, 1, nbalizas)])
% R = diag(var_med_us)

%% Dibujos
for i = 1:N_sensores_us
    figure;
    hold on
    plot(1:N_medidas, medidas_us(:, i));
    xlabel('z (m)');
    ylabel('medida')
    title("ultrasonidos #" + num2str(i));
end

figure;
plot(1:N_medidas, medidas_ls(:, 1));
xlabel('z (rad)');
ylabel('medida')
title('ángulo con baliza con láser');

% figure;
% plot(1:N_medidas, medidas_ls(:, 2));
% xlabel('z (m)');
% ylabel('medida')
% title('distancia a baliza con láser');

save calibracion_sensores R