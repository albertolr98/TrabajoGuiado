%% apoloGetLocation 
%  retorna x y z roll pitch yaw (angulos en radianes)
apoloGetLocation('Marvin','MiMundo')

%% apoloGetLocationMRobot
%  retorna x y z yaw (Mejor para nuestro caso)
apoloGetLocationMRobot('Marvin','MiMundo')

%% apoloMoveMRobot(robot,speeds,time,world) 
%  speeds = [avance rotacion]
%  mueve el robot a las velocidades seleccionadas el tiempo seleccionado
apoloMoveMRobot('Marvin',[0.2 -0.2],0.1, 'MiMundo')
apoloUpdate()

%% apoloPlace (object, pos, orientacion, world)
% No comprueba colisiones con el entorno

apoloPlace('Marvin',[0,0,0],[0,0,0],'MiMundo')
apoloUpdate()

%% apoloPlaceMRobot(robot, pose, angle, world)
% Igual que el anterior pero solo el angulo de rotación
% Además comprueba que no colisione con el entorno

apoloPlaceMRobot('Marvin',[0,0,0],0,'MiMundo')
apoloUpdate()

%% apoloPlaceXYZ(object,x,y,z,world)
% Posiciona un objeto choque o no
% Cuidao que puedes poner el bicho flying

apoloPlaceXYZ('Marvin',2,1,0,'MiMundo')
apoloUpdate()


%% apoloGetAllultrasonicSensors(object,world)
readings = apoloGetAllultrasonicSensors('Marvin','MiMundo');
for i =1:30
    readings_in = apoloGetAllultrasonicSensors('Marvin','MiMundo');
    readings = [readings ; readings_in];
    apoloUpdate()
end

% Calculo varianza

sensor1 = readings(:,1);
var(sensor1)


%% apoloGetLaserData(Laser, world)
% Mazo de datos
apoloGetLaserData('LMS100', 'MiMundo')


%% apoloGetLaserLandMarks(laser,world) 
% Devuelve un struct con id angle y distancia
apoloGetLaserLandMarks('LMS100', 'MiMundo')


%% apoloGetOdometry(robot, world)
% devuelve [x,  y , theta]
apoloGetOdometry('Marvin','MiMundo')


%% apoloGetUltrasonicSensor(sensor,world)
% Devuelve la medida de un solo sensor
apoloGetUltrasonicSensor('uc0','MiMundo')


%% apoloResetOdometry(robot,pose,world)
% resetea la odometria y le dice al robot donde esta [pose]

apoloResetOdometry('Marvin',[0 2 0.2],'MiMundo')