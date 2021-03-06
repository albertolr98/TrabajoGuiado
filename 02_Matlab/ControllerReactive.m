function [v,w,counter,reached, derecha] = ControllerReactive(pos_objetivo,pos_robot,counter,orientacion_choque,derecha)
    %Control reactivo: si el piloto decide activar el modo control reactivo
    %se inicia una secuencia de movimientos:
    %   1. Marcha atras
    %   2. Giro (sentido depende de sensores)
    %   3. Avance

    % Medición con sensores
    medidas_us = GetUltrasonicSensorsWithNoise('Marvin');
   
    sensor_izq = medidas_us(2);
    sensor_derecho = medidas_us(5);



reached = 0;

% Evita error de no asignación
derecha = derecha;


% Secuencia de movimientos guiada por contador
if counter == 0
    if(sensor_izq<sensor_derecho) && sensor_izq<2.9
        derecha = 1;
    else
        derecha = 0;
    end
    v = 0;
    w = 0;
else if counter < 50
    v=-0.1;
    w=0;
elseif counter < 175
    v = 0;
    if(derecha)
        w = -0.1;
    else
        w = 0.1;
    end
elseif counter< 240
    v = 0.1;
    w = 0;
else
    reached = 1;
    v = 0;
    w = 0;
end


end

