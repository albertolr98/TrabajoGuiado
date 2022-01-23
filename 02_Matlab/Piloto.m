function [modo] = Piloto(choque)
    %PILOTO Si ve que hay un obstaculo cerca activa modo reactivo
    % Modo 1 -> modo normal
    % Modo 2 -> reactivo

    % Recogida medidas de los sensoressensores 
    medidas_us = GetUltrasonicSensorsWithNoise('Marvin');
   
    sensor_izq = medidas_us(2);
    sensor_derecho = medidas_us(5);
    sensor_frontal = medidas_us(1);
    sensor_izq_trasero = medidas_us(3);
    sensor_der_trasero = medidas_us(4);

    % Decisión modo
    modo = 1;
    if sensor_frontal <0.3 || choque == 0
        modo = 2;
    end

end

