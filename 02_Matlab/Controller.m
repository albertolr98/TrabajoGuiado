function [v,w,mode,reached] = Controller(pos_objetivo,pos_robot,mode,choque)
    %CONTROLLER esta funcion toma la posicion relativa a la cual 
    %debe ir el robot y proporciona v y w para alcanzar el objetivo
    %Como de momento pos_robot es con getLocation tiene la forma [x y z
    %theta]
    
%     tol_giro = 0.05;
%     tol_distancia = 0.01;
    tol_giro = 0.1;
    tol_distancia = 0.1;

    %Calculo de diferencia de posicion 
    pos_dif =pos_objetivo([1,2])-pos_robot([1,2]);
    angle_robot = wrapToPi(pos_robot(3));

    %Calculo distancia
    distancia = sqrt(pos_dif(1)^2+pos_dif(2)^2);
    
    %Calculamos angulo entre robot y objetivo
    angle_objetivo =wrapToPi(atan2(pos_dif(2),pos_dif(1))); 
    angle_dif = wrapToPi(angle_objetivo - angle_robot);
    
    %Una vez llega a la posicion se reorienta 

    %orientation_dif = wrapToPi(pos_objetivo(3)-angle_robot);
    orientation_dif = 0;

    %El tipo de control que he implementado es un proporcional bastante
    %cutre pero funciona, se puede mejorar pero creo que es más interesante
    %centrarse en lo que aún no funciona
    
  
    %Correccion angulo
    if ~choque
        v = -0.1 +(0.1+0.1).*rand(1,1);
        w = -0.1 +(0.1+0.1).*rand(1,1);
        mode = 1;
        reached = 0;

    elseif (abs(angle_dif) > tol_giro) && mode~=3
        %Como al llegar al objetivo el angulo se puede volver loco he
        %puesto que si ha llegado al modo 3 "Correccion orientacion" no
        %pueda regresar al modo correccion de ángulo. Seguramente esto se
        %pueda poner como una variable global para no tener que
        %realimentarlo y tal
        
        mode = 1;
        w = angle_dif/pi * 0.2;  %Valor entre 0-1 por eso divido entre pi
                       %Dado que el maximo error va a ser pi radianes
        v = 0;
        reached = 0;
    
    %Correccion distancia
    elseif (abs(pos_dif(1))>tol_distancia) || (abs(pos_dif(2))>tol_distancia)
        mode = 2;
        v = distancia;
        w = 0;
        reached = 0;
    
    %Correccion orientacion
    elseif abs(orientation_dif)>tol_giro
        if mode~=3
            disp("MODO3")
        end
         mode = 3;
         v = 0;
         w = orientation_dif/pi;
         reached = 0;
    %aparcao
    else
         mode = 4;
         v = 0;
         w = 0;
         reached = 1;
    end

end

