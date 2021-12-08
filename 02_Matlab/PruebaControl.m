%% Prueba para hacer control como dice Peris

%Variables
i = 0;
iteraciones = 10000;
robot = 'Marvin';

n_fases = 12;
ref_pos = [ 1,  4,  0;
            7,  4 ,  -pi/2;
            7,  1 ,  pi;
            3,  1 ,  pi/2;
            3,  4 ,  pi;
            1,  4,  pi;
            1,  5.75,  0;
            7,  5.75,  pi;
            1,  5.75,  pi/2;
            0.75,  10,  pi/2;
            0.75,  15,  pi/2;
            3.5,  17.5,  0;
            ];
        

start_pos = [1, 1, pi/2];

%Inicializacion arrays para plotear
v_array = 0;
w_array = 0;
reached_array = 0;
mode_array = 1;
mode = 1;


%%Posicionamos a tito marvin para las pruebas
    apoloPlaceMRobot(robot,[start_pos([1,2]),0],start_pos(3));    
    apoloResetOdometry(robot,[0,0,0]);
    apoloUpdate();
    

    fase = 1;
    pos_robot_array = apoloGetLocationMRobot(robot);
while i< iteraciones && fase<=n_fases
    %% GetLocation
    pos_robot = apoloGetLocationMRobot(robot);
    
    
    %% Debugging
    %fprintf("angle_robot: %f angle_obj: %f angle_resta: %f\n",radtodeg(angle),radtodeg(wrapToPi(atan2(pos(2),pos(1)))),radtodeg(angle_dif));
   
    %% Controlador
    [v,w,mode,reached] = Controller(ref_pos(fase,:),pos_robot,mode);
    
    %Recogemos en un array las variables para hacer un plot
    v_array = [v_array; v];
    w_array = [w_array; w];
    mode_array = [mode_array; mode];
    reached_array = [reached_array; reached];
    pos_robot_array = [pos_robot_array; pos_robot];


    %% Mover Robot
    apoloMoveMRobot(robot,[v, w],0.1);
    apoloUpdate();

    
    %% Si completa el objetivo pasa al siguiente
    if reached
        fase = fase + 1;
        reached = 0;
        apoloGetLocationMRobot(robot);
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
 plot(pos_robot_array(:,1),pos_robot_array(:,2),'b-');
 title("Trayectoria 2d");
 

apoloGetLocationMRobot(robot)
