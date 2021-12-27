%% Prueba para hacer control como dice Peris
%Variables
i = 0;
iteraciones = 1000;
robot_name = 'Marvin';

% Contrucción del entorno
en = entorno;
en = add_pared(en, [0 0], [0 19.4]);
en = add_pared(en, [0 19.4], [8.0 19.4]);
en = add_pared(en, [8.0 19.4],[8.0 0]);
en = add_pared(en, [8.0 0],[0 0]);

en = add_pared(en, [2.0 0],[2.0 3.2]);
en = add_pared(en, [2.0 4.6],[2.0 5.0]);
en = add_pared(en, [2.0 6.4],[2.0 8.6]);

en = add_pared(en, [2.0 4.6],[8.0 4.6]);
en = add_pared(en, [2.0 6.8],[8.0 6.8]);

en = add_pared(en, [1.4 10.0],[8.0 10.0]);
en = add_pared(en, [1.4 14.6],[8.0 14.6]);

ref_pos =  [1.0000 7.0000 7.0000 3.0000 3.0000 1.0000 1.0000 7.0000 1.0000 0.7500 0.7500 3.5000;
            4.0000 4.0000 1.0000 1.0000 4.0000 4.0000 5.7500 5.7500 5.7500 10.000 15.000 17.5000;
            0.0000 -pi/2  pi     pi/2   pi     pi     0      pi     pi/2   pi/2   pi/2   0       ];

n_fases = size(ref_pos,2);

start_pos = [1; 1; pi/2];

Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Pk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];

% Construcción del robot
bot = robot(start_pos);
bot = add_us(bot, [0.2 0 0]);
bot = add_us(bot, [0.18 0.11 0.7]);
bot = add_us(bot, [0.18 -0.11 -0.7]);

%Inicializacion arrays para plotear
v_array = 0;
w_array = 0;
reached_array = 0;
mode_array = 1;
mode = 1;


%%Posicionamos a tito marvin para las pruebas
apoloPlaceMRobot(robot_name,[start_pos(1) start_pos(2) 0], start_pos(3));    
apoloResetOdometry(robot_name,[0,0,0]);
apoloUpdate();


fase = 1;
pos_robot_array = start_pos;
pos_robot = start_pos;

%% Debugging porque me quiero morir
Zk_ = [0;0;0];
deb = [0;0;0];

%% Bucle como tal
while i< iteraciones && fase<=n_fases
    %% Debugging
    %fprintf("angle_robot: %f angle_obj: %f angle_resta: %f\n",radtodeg(angle),radtodeg(wrapToPi(atan2(pos(2),pos(1)))),radtodeg(angle_dif));
   
    %% Controlador
    %[v,w,mode,reached] = PruebaController(ref_pos(:,fase),pos_robot,mode);
    v = 0.2;
    w = 0;
    mode = 1;
    reached = 0;

    %Recogemos en un array las variables para hacer un plot
    v_array = [v_array; v];
    w_array = [w_array; w];
    mode_array = [mode_array; mode];
    reached_array = [reached_array; reached];
    pos_robot_array = [pos_robot_array, pos_robot];

    % Debug
    deb = [deb, Zk_];
    
    %% Mover Robot
    apoloMoveMRobot(robot_name,[v w],0.1);
    apoloUpdate();

    %% Filtro de Kalman
    [pos_robot, Pk, Zk_] = ConLaKalman(pos_robot, Pk, [v w], bot, en);

    %% Si completa el objetivo pasa al siguiente
    if reached
        fase = fase + 1;
        reached = 0;
        apoloGetLocationMRobot(robot_name);
    end
    i = i + 1;
    pause(1/1000);
    
end

 x=[1:1:(i+1)];
 
 figure("Name","Velocidades");
 subplot(2,2,1);
 plot(x,v_array, 'b-');
 title("velocidad lineal");
 
 subplot(2,2,2);
 plot(x,w_array, 'b-');
 title("velocidad angular");
 
 subplot(2,2,3);
 plot(x,mode_array, 'b-');
 title("Modo de control");
 
 subplot(2,2,4);
 plot(x,reached_array, 'b-');
 title("Reached target");
 
 
 figure("Name", "Posicion 2d");
 plot(pos_robot_array(1,:),pos_robot_array(2,:),'b-');
 title("Trayectoria 2d");
 

apoloGetLocationMRobot(robot_name)
