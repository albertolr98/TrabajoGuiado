% OdometryCalibration([7,2.3],pi,0)

%function [error, straight_loc, straight_odom] = OdometryCalibration(position,angle,velocity)
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

robot = 'Marvin';
position = [-10,10];
angle = pi/2;
n = 500;


%posicionamos Marvin en el origen
apoloPlaceMRobot(robot,[position,0],angle);
apoloUpdate()

%Reseteamos su odometria 
%ESTA POSICION RARA POR EL FALLO DE POSICIONAMIENTO
apoloResetOdometry(robot,[position, angle])

%Movimiento en linea recta
pos=apoloGetOdometry(robot);
loc=apoloGetLocationMRobot(robot);

%Acumulo valores de odometria y de localizacion del robot
straight_odom=[pos];
straight_loc=[loc];


for i = 1:n
    apoloMoveMRobot(robot, [0.1, 0.1], 0.05); 
    apoloUpdate() 
    
    a=apoloGetOdometry(robot); 
    b=apoloGetLocationMRobot(robot);
    
    straight_odom =[straight_odom ; a]; 
    straight_loc = [straight_loc ; b];
end 

%Como la Z no me interesa tomo solo el valor de [x y angle]
straight_loc = [straight_loc(:,1) straight_loc(:,2) straight_loc(:,4)];

%El error es la medida de la odometria menos la localizacion
error= straight_odom-straight_loc; 

%Desplazamos el vector del error 1 posición para poder restarlo al error
error_anterior=[[0 0 0]; error(1:n,:)];

%Como los de la valores de la odometría van creciendo para centrarlos sobre
%0 se le resta el valor del error anterior
error = error - error_anterior;

%Ploteamos
figure('Name','Error');
subplot(1,3,1); plot(2:n,error(3:end,1)); title('X Error'); %Error X
subplot(1,3,2); plot(2:n,error(3:end,2)); title('Y Error'); %Error Y no tiene error porque no gira 
subplot(1,3,3); plot(2:n,error(3:end,3)); title('Angle Error'); %Error angle es un caos porque alterna entre 0 y 2pi

%end
