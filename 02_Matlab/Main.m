%% Main
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
% Hacer bucle infinito con cosas. 
%#ok<*NUSED>
clear
global time_unit 

load calibracion_odometria % Q_pu
variables_globales

%% Calibracion
Calibracion % se calibran los sensores
Calibracion_odometria % se calibra la odometria