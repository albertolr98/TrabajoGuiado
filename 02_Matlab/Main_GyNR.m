%% Main
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
clearvars
clc

global time_unit robot_name %#ok<*NUSED>

%% Variables
i = 0;
iteraciones = 100000;

variables_globales
%% Contrucción del entorno
load construccion_entorno_robot

%% Posiciones objetivo
ref_pos =  [1.0000 7.0000 7.0000 3.0000 3.0000 1.0000 1.0000 7.0000 1.0000 0.7500 0.7500 3.5000;
            4.0000 4.0000 1.0000 1.0000 4.0000 4.0000 5.7500 5.7500 5.7500 10.000 15.000 17.5000;
            0.0000 -pi/2  pi     pi/2   pi     pi     0      pi     pi/2   pi/2   pi/2   0   ];

n_fases = size(ref_pos, 2);

%% Inicialización
start_pos = [1; 1; pi/2];

Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

fase = 1;
X_estimada = start_pos;
X_estimada_array = start_pos;
X_real_array = start_pos;

% Construcción del robot
bot = robot(start_pos);
bot = add_us(bot, [0.2 0 0]);
bot = add_us(bot, [0.18 0.11 0.7]);
bot = add_us(bot, [0.18 -0.11 -0.7]);

% Inicializacion arrays para plotear
v_array = 0;
w_array = 0;
reached_array = 0;
mode_array = 1;
mode = 1;

% Posicionamiento del robot
apoloPlaceMRobot(robot_name,[start_pos(1) start_pos(2) 0], start_pos(3));    
apoloResetOdometry(robot_name,[0,0,0]);
apoloUpdate();

<<<<<<< HEAD:02_Matlab/Main.m
au = 1; % variable para no hacer apoloUpdate todo el rato
=======
%% Debugging
% Zk_ = [0;0;0];
% deb = [0;0;0];
>>>>>>> a0615665394be6a775b47d13c284492a88dee5cf:02_Matlab/PruebaPruebaControl.m

%% Bucle como tal
while i< iteraciones && fase<=n_fases
    %% Controlador
    [v,w,mode,reached] = Controller(ref_pos(:,fase),X_estimada,mode);

    % solo para hacer comparaciones
    X_real = apoloGetLocationMRobot(robot_name);
    
    % Recogemos en un array las variables para hacer un plot
    v_array = [v_array; v];
    w_array = [w_array; w];
    mode_array = [mode_array; mode];
    reached_array = [reached_array; reached];
    X_estimada_array = [X_estimada_array, X_estimada];
    X_real_array = [X_real_array, [X_real(1); X_real(2); X_real(4)]];
        
    %% Movimiento Robot
    apoloResetOdometry(robot_name, [0 0 0]) 
    apoloMoveMRobot(robot_name,[v w],0.1);
    
    %% Filtro de Kalman
    [X_estimada, Pk] = KalmanFilter(X_estimada, Pk, [v w], bot, en);

    bot = bot.actualizar_posicion(X_estimada);
    
    %% Si completa el objetivo pasa al siguiente
    if reached
        fase = fase + 1;
        reached = 0;
        apoloGetLocationMRobot(robot_name);
    end
    i = i + 1;
    
    if au == 10
        au = 1;
        apoloUpdate();
    else
        au = au + 1;
    end
    
    
end

%% Dibujos de interés
t=1:i+1;

figure("Name","Velocidades");
subplot(2,2,1);
plot(t,v_array, 'b-');
title("velocidad lineal");

subplot(2,2,2);
plot(t,w_array, 'b-');
title("velocidad angular");

subplot(2,2,3);
plot(t,mode_array, 'b-');
title("Modo de control");

subplot(2,2,4);
plot(t,reached_array, 'b-');
title("Reached target");

%%
figure("Name", "Posicion 2d");
hold on
plot(X_estimada_array(1,:),X_estimada_array(2,:),'b-');
plot(X_real_array(1,:),X_real_array(2,:),'r-');
plot(ref_pos(1,:), ref_pos(2,:), 'xm');
plot_entorno(en, '-k', 'LineWidth', 2)
xlim([-6 14]);
ylim([0 20]);

title("Trayectoria 2d");


apoloGetLocationMRobot(robot_name)
