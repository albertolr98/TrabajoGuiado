function [v,w,counter,reached, derecha] = ControllerReactive(pos_objetivo,pos_robot,counter,orientacion_choque,derecha)
    %CONTROLLER esta funcion toma la posicion relativa a la cual 
    %debe ir el robot y proporciona v y w para alcanzar el objetivo
    %Como de momento pos_robot es con getLocation tiene la forma [x y z
    %theta]

    
    medidas_us = GetUltrasonicSensorsWithNoise('Marvin');
   
    sensor_izq = medidas_us(2);
    sensor_derecho = medidas_us(5);
    sensor_frontal = medidas_us(1);
    sensor_izq_trasero = medidas_us(3);
    sensor_der_trasero = medidas_us(4);


reached = 0;
derecha = derecha;
if counter == 0
    if(sensor_izq<sensor_derecho)
        derecha = 1;
    else
        derecha = 0;
    end
    v = 0;
    w = 0;
else if counter < 50
    v=-0.1;
    w=0;
elseif counter < 150
    v = 0;
    if(derecha)
        w = -0.1;
    else
        w = 0.1;
    end
elseif counter< 200
    v = 0.1;
    w = 0;
else
    reached = 1;
    v = 0;
    w = 0;
end
    
%     if counter > 10
%         reached  = 1;
%     end

end

