function [v,w,mode,reached] = Controller(robot,pos_objetivo,pos_robot,mode)
    %CONTROLLER esta funcion toma la posicion relativa a la cual 
    %debe ir el robot y proporciona v y w para alcanzar el objetivo
    %Como de momento pos_robot es con getLocation tiene la forma [x y z
    %theta]
    
    tol_giro = 0.05;
    tol_distancia = 0.01;

    %Calculo de diferencia de posicion
    pos_dif =pos_objetivo([1,2])-pos_robot([1,2]);
    angle_robot = wrapToPi(pos_robot(4));
    orientation_dif = wrapToPi(pos_objetivo(3)-angle_robot);

    distancia = sqrt(pos_dif(1)^2+pos_dif(2)^2);
    
    %Calculamos angulo entre robot y objetivo
    angle_objetivo =wrapToPi(atan2(pos_dif(2),pos_dif(1))); 
    angle_dif = wrapToPi(angle_objetivo - angle_robot);
    
  
    %Correccion angulo
    if (abs(angle_dif) > tol_giro) && mode~=3
        %disp('Mode 1');
        mode = 1;
        w = angle_dif/pi;  %Valor entre 0-1 por eso divido entre pi
                       %Dado que el maximo error va a ser pi radianes
        v = 0;
        reached = 0;
    
    %Correccion distancia
    elseif (abs(pos_dif(1))>tol_distancia) || (abs(pos_dif(2))>tol_distancia)
        %disp('Mode 2');
        mode = 2;
        v = distancia;
        w = 0;
        reached = 0;
    
    %Correccion orientacion
    elseif abs(orientation_dif)>tol_giro
         %disp('Mode 3');
         mode = 3;
         v = 0;
         w = orientation_dif/pi;
         reached = 0;
    %aparcao
    else
         %disp('Aparcao');
         mode = 4;
         v = 0;
         w = 0;
         reached = 1;
    end

end

