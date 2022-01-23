%% Main
% Trabajo de Guiado y Navegación de Robots
% Pablo García Peris, Guillermo Illana Gisbert y Alberto López Rodríguez
clearvars
clc

global time_unit robot_name nbalizas %#ok<*GVMIS,*NUSED>

%% Variables
iteraciones = 30000;
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

% [X_estimada, Pk] = EMC(bot, en, 1); % 100 iteraciones, "en" es una objeto del tipo entorno
% 
% for i = 1:100
%     [X_estimada, Pk] = KalmanFilter(X_estimada, Pk, [0 0], bot, en);
% 
%     bot = bot.actualizar_posicion(X_estimada);
% end

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
orientacion_choque = 0;
reached = 0;
counter = 0;
reactivo = 0;
modo = 0;
derecha = 0;
% n_fases = 0;
nu = [0,0,0,0,0];
% Posicionamiento del robot
apoloPlaceMRobot(robot_name,[start_pos(1) start_pos(2) 0], start_pos(3));    
apoloResetOdometry(robot_name,[0,0,0]);
apoloUpdate();

au = 1; % variable para no hacer apoloUpdate todo el rato

%% Planificacion
% inflacion = 0.5;
%resolucion = 0.1;
inflacion = 0.5;
resolucion = 0.1;
ref_pos = trayectoria(1,:)';

%% Calculo trayectoria 
for i = 1:size(trayectoria,1)-1
    ref_pos_i = Planner(trayectoria(i,:),trayectoria(i+1,:),resolucion,inflacion, en);
    
    % Almacenamos la trayectoria total
    ref_pos = [ref_pos,ref_pos_i];

end
n_fases = size(ref_pos, 2);


%% Bucle principal
i = 0;

while i< iteraciones && fase<=n_fases  

    if fase == n_fases
        control_orientacion = 1;
    end
    %% Piloto
    modo = Piloto(choque);
    %% Controlador
    if modo == 1 && reactivo == 0
        [v,w,mode,reached] = Controller(ref_pos(:,fase),X_estimada,mode,reached,control_orientacion);
        counter = 0;
    else
        
        [v,w,counter,reached,derecha] = ControllerReactive(ref_pos(:,fase),X_estimada,counter,orientacion_choque,derecha);
        counter = counter + 1;
        reactivo = 1;          
    end
    
    %Para desactivar el control reactivo
    if counter>0 && reached == 1
        reactivo = 0;
        reached = 0;
    end
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
    choque = apoloMoveMRobot(robot_name,[v w],0.1);
        
    
    
    %% Filtro de Kalman
    [X_estimada, Pk, nu] = KalmanFilter(X_estimada, Pk, [v w], bot, en);

    bot = bot.actualizar_posicion(X_estimada);

    %% Si completa el objetivo pasa al siguiente

    salto_fases = 3;

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
