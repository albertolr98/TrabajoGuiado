%% Prueba para hacer control como dice Peris

%Variables
i = 0;
iteraciones = 10000;
robot = 'Marvin';
ref_pos = [ -4,  5 ,  pi;
            -6,  5 ,  -pi/2;
            -6,  3 ,  0;
            -4,  3 ,  pi/2];
ref_angle = 0;
start_pos = [-4, 3, pi];

v_array = 0;
w_array = 0;
reached_array = 0;
mode_array = 1;



%%Posicionamos a tito marvin para las pruebas
    apoloPlaceMRobot(robot,[-4,3,0],pi);    
    apoloResetOdometry(robot,[0,0,0]);
    apoloUpdate();
    pos_theta = [0,0,0];
    fase = 1;
while i< iteraciones && fase<5
    %% GetLocation
    pos_robot = apoloGetLocationMRobot(robot);
%     pos =pos_temp([1,2]);
%     angle = wrapToPi(pos_temp(4));
%     
%     pos_dif = ref_pos - pos;
%     angle_dif = wrapToPi(wrapToPi(atan2(ref_pos(2)-pos(2),ref_pos(1)-pos(1))) - angle);
%     
%     pos_controller = [pos_dif, angle_dif];
%     pos_theta = [pos_theta; [angle, wrapToPi(atan2(pos(2),pos(1))),angle_dif]];
%     
    %% Debugging
    %fprintf("angle_robot: %f angle_obj: %f angle_resta: %f\n",radtodeg(angle),radtodeg(wrapToPi(atan2(pos(2),pos(1)))),radtodeg(angle_dif));
   
    %% Controlador
    [v,w,mode,reached] = Controller(robot,ref_pos(fase,:),pos_robot,mode);
    v_array = [v_array; v];
    w_array = [w_array; w];
    mode_array = [mode_array; mode];
    reached_array = [reached_array; reached];


    %% Mover Robot
    apoloMoveMRobot(robot,[v, w],0.1);
    
    apoloUpdate();

    
    %% Si completa el objetivo pasa al siguiente
    if reached
        fase = fase + 1;
        disp(fase);
        reached = 0;
        apoloGetLocationMRobot(robot)
    end
    i = i + 1;
    pause(10/1000);
    
end



 x=[1:1:(i+1)];
 subplot(4,1,1);
 plot(x,v_array, 'b-');
 subplot(4,1,2);
 plot(x,w_array, 'b-');
 subplot(4,1,3);
 plot(x,mode_array, 'b-');
 subplot(4,1,4);
 plot(x,reached_array, 'b-');

apoloGetLocationMRobot(robot)
