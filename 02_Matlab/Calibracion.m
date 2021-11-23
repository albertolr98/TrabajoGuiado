apoloPlaceMRobot('Marvin',[1,1,0],-pi/2,'World 1')
apoloUpdate()

medidas = apoloGetAllultrasonicSensors('Marvin','World 1');
odometria = apoloGetOdometry('Marvin','World 1');
for i=1:100
    medidas_next = apoloGetAllultrasonicSensors('Marvin','World 1');
    odometria_next = apoloGetOdometry('Marvin','World 1');

    apoloMoveMRobot('Marvin',[-0.2 0.1],0.05, 'World 1');
    apoloUpdate()

    medidas = [medidas; medidas_next];
    odometria = [odometria; odometria_next];
end

var(medidas)
var(odometria)