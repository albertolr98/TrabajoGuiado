function [modo] = Piloto(nu,modo)
%PILOTO Summary of this function goes here
%   Detailed explanation goes here
% Modo 1 -> modo normal
% Modo 2 -> reactivo
    medidas_us = GetUltrasonicSensorsWithNoise('Marvin');
   
    sensor_izq = medidas_us(2);
    sensor_derecho = medidas_us(5);
    sensor_frontal = medidas_us(1);
    sensor_izq_trasero = medidas_us(3);
    sensor_der_trasero = medidas_us(4);



    modo = 1;
    if sensor_frontal <0.3%||abs(nu(2))>2||abs(nu(3))>2%||abs(nu(4))>2||abs(nu(5))>2
        modo = 2;
    end

end

