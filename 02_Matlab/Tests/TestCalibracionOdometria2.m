%% Test de calibración de odometría2  (incremental)
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
format longG
global time_unit

load calibracion_odometria % Q_pi

%% Coloación del robot
N_medidas = 100;
N_sensores = 3;

apoloUpdate();

%% Pruebas
v = 0.2;
w = 0.6;

x = -12;
y = 10;
theta = -pi/4;

%% Incremental
odometria_inc = zeros(N_medidas, 3);

M = [cos(theta) -sin(theta)  0;
     sin(theta) cos(theta)   0;
     0          0            1]; % matriz de rotación

for i = 1:N_medidas
    apoloPlaceMRobot('Marvin', [x y 0], theta, 'World 1');
    apoloResetOdometry('Marvin', [0 0 0], 'World 1');

    apoloMoveMRobot('Marvin', [v w], time_unit, 'World 1');
    apoloUpdate();
    
    odometria_next = apoloGetOdometry('Marvin','World 1');

    odometria_inc(i, :) = [x y theta] + (M*odometria_next')';
end

media_inc = mean(odometria_inc)

%% Directamente
odometria_abs = zeros(N_medidas, 3);

M = [cos(theta) -sin(theta)  0;
     sin(theta) cos(theta)   0;
     0          0            1]; % matriz de rotación

for i = 1:N_medidas
    apoloPlaceMRobot('Marvin', [x y 0], theta, 'World 1');
    apoloResetOdometry('Marvin', [x y theta], 'World 1');

    apoloMoveMRobot('Marvin', [v w], time_unit, 'World 1');
    apoloUpdate();
    
    odometria_next = apoloGetOdometry('Marvin','World 1');

    odometria_abs(i, :) = odometria_next;
end

media_abs = mean(odometria_abs)

%% Resultado real

real_1 = apoloGetLocationMRobot('Marvin', 'World 1');
real = real_1([1 2 4])

%% Resultados
figure(10);
plot(odometria_inc(:, 1));
title("medidas de la odometría " + num2str(1));



