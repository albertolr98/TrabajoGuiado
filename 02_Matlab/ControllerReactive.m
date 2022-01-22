function [v,w,counter,reached] = ControllerReactive(pos_objetivo,pos_robot,counter,orientacion_choque)
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

    orientation_dif = wrapToPi(orientacion_choque-angle_robot);
    %orientation_dif = 0;

    %El tipo de control que he implementado es un proporcional bastante
    %cutre pero funciona, se puede mejorar pero creo que es más interesante
    %centrarse en lo que aún no funciona
    
    medidas_us = GetUltrasonicSensorsWithNoise('Marvin');
    disp(' ')
    sensor_izq = medidas_us(2)
    sensor_derecho = medidas_us(5)
    sensor_frontal = medidas_us(1)
    sensor_izq_trasero = medidas_us(3)
    sensor_der_trasero = medidas_us(4)

    %Correccion angulo
      v = -0.2 + (0.3).*rand(1,1);
      w = -0.4 + (0.3).*rand(1,1);
%     if sensor_frontal < 0.5
%         v = -0.5
%         w = -0.1 + (0.2).*rand(1,1)
%     elseif (sensor_izq > sensor_derecho) && (sensor_frontal < 2)
%         v = 0
%         w = -0.2
%     elseif (sensor_izq < sensor_derecho) && (sensor_frontal < 2) 
%         v = 0
%         w = 0.2
%     elseif (sensor_frontal > 2)
%         v = 0.1;
%         w = 0;
%     else 
%         v = -0.1 + (0.2).*rand(1,1);
%         w = -0.1 + (0.2).*rand(1,1);
%  
%     end
    
    reached = 0;
    if counter > 10
        reached  = 1
    end

end

