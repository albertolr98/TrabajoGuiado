%% Main
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
clearvars
clc

global time_unit robot_name nbalizas %#ok<*GVMIS,*NUSED>

%% Variables
iteraciones = 100000;
variables_globales

%% Contrucción del entorno
load construccion_entorno_robot

%% Inicialización
start_pos = [1; 1; pi/2];
trayectoria = [start_pos';
                6,13,0;
                start_pos';
                6,6,0];

bot = bot.actualizar_posicion(start_pos); % bot es una objeto del tipo "robot"

Pk = diag(ones(1,3))*0.001;

% Posición inicial y arrays para guardar las posiciones reales y estimadas
X_estimada = start_pos;
X_estimada_array = X_estimada;
X_real_array = start_pos;

fase = 1;

% Inicializacion arrays para plotear
v_array = 0;
w_array = 0;
reached_array = 0;
mode_array = 1;
mode = 1;
choque = 1;
control_orientacion = 0;
reached = 0;
% n_fases = 0;

% Posicionamiento del robot
apoloPlaceMRobot(robot_name,[start_pos(1) start_pos(2) 0], start_pos(3));    
apoloResetOdometry(robot_name,[0,0,0]);
apoloUpdate();

au = 1; % variable para no hacer apoloUpdate todo el rato

%% Planificacion
inflacion = 0.5;
resolucion = 0.2;


ref_pos = trayectoria(1,:)';
for i = 1:size(trayectoria,1)-1
    %while n_fases == 0
        ref_pos_i = Planner(trayectoria(i,:),trayectoria(i+1,:),resolucion,inflacion, en);
%         disp(ref_pos_i)
        %n_fases = size(ref_pos_i, 2);
   % end
    ref_pos = [ref_pos,ref_pos_i];

end
n_fases = size(ref_pos, 2);

%% Posiciones objetivo
% ref_pos =  [1.0000 7.0000 7.0000 3.0000 3.0000 1.0000 1.0000 7.0000 1.0000 0.7500 0.7500 3.5000;
%             4.0000 4.0000 1.0000 1.0000 4.0000 4.0000 5.7500 5.7500 5.7500 10.000 15.000 17.5000;
%             0.0000 -pi/2  pi     pi/2   pi     pi     0      pi     pi/2   pi/2   pi/2   0   ];

% ref_pos =  repmat([-5 -15 -15 -5 ;
%             15 15 5 5 ;
%             0 -pi/2 pi pi/2], 1, 30);
% 
% n_fases = size(ref_pos, 2);



%% Posiciones objetivo
% ref_pos =  [1.0000 7.0000 7.0000 3.0000 3.0000 1.0000 1.0000 7.0000 1.0000 0.7500 0.7500 3.5000;
%             4.0000 4.0000 1.0000 1.0000 4.0000 4.0000 5.7500 5.7500 5.7500 10.000 15.000 17.5000;
%             0.0000 -pi/2  pi     pi/2   pi     pi     0      pi     pi/2   pi/2   pi/2   0   ];
% 
% n_fases = size(ref_pos, 2);

%% Bucle como tal
i = 0;

while i< iteraciones && fase<=n_fases  
    %% Controlador
    if fase == n_fases
        control_orientacion = 1;
    end
    [v,w,mode,reached] = Controller(ref_pos(:,fase),X_estimada,mode,choque,reached,control_orientacion);

    % solo para hacer comparaciones
    X_real = apoloGetLocationMRobot(robot_name);
    
    % Recogemos en un array las variables para hacer un plot
    v_array = [v_array; v]; %#ok<*AGROW> 
    w_array = [w_array; w];
    mode_array = [mode_array; mode];
    reached_array = [reached_array; reached];
    X_estimada_array = [X_estimada_array, X_estimada];
    X_real_array = [X_real_array, [X_real(1); X_real(2); X_real(4)]];
        
    %% Movimiento Robot
    apoloResetOdometry(robot_name, [0 0 0]) 
    choque = apoloMoveMRobot(robot_name,[v w],0.1);
    
    %% Filtro de Kalman
    [X_estimada, Pk] = KalmanFilter(X_estimada, Pk, [v w], bot, en);

    bot = bot.actualizar_posicion(X_estimada);

    %% Si completa el objetivo pasa al siguiente

    salto_fases = 1;

    if reached
        if fase < (n_fases-salto_fases)
            fase = fase + salto_fases;
        elseif fase < n_fases
            fase = n_fases;
        elseif fase == n_fases
            fase = fase+1;
        end
        reached = 0;
    end
    i = i + 1;
    

    % Hace apolo update solo cada 2 iteraciones
    if au == 1
        apoloUpdate();
        au = au +1;
    elseif au == 2
        au = 1;
    else 
        au = au +1;
    end
    

    
end

%% Dibujos de interés
t=0:i;

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
plot(trayectoria(:,1), trayectoria(:,2), 'xm');
plot_entorno(en)
legend('trayectoria estimada', 'trayectoria real')
xlim([-6 14]);
ylim([0 20]);

title("Trayectoria 2d");


% apoloGetLocationMRobot(robot_name)
