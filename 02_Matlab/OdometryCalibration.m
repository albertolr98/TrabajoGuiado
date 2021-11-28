function  OdometryCalibration(position,angle,velocity)
% ODOMETRYCALIBRATION 
% Parametros:
%   -Position[x,y]:   posición robot en el mapa
%   -Angle[rad]:      angulo robot en el mapa
%   -Velocity[m/s]:   velocidad a la que se mueve el robot

% Esta función deberia estimar los errores presentes en
% la odometria. Para ello:
%   1. Se coloca al robot en un punto conocido del mapa
%   2. Se le hace avanzar la misma distancia varias veces y se pregunta por la
%      posición. 

% Como hay varias variables que pueden influir en la odometría
% se analizan varias trayectorias: 
%   1. Linea recta
%   2. Giro en sitio
%   3. Avance y giro

end
