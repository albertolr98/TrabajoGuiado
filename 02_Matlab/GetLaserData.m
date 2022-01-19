function Z = GetLaserData(laser_name, nbalizas)
%Z=GETLASERDATA(laser_name, nbalizas)
% Obtiene las medidas de ángulo de las balizas (apoloGetLaserLandMarks) 

laser = apoloGetLaserLandMarks(laser_name);
id = laser.id;
ang = laser.angle;
dist = laser.distance;
Z = [];
for i = 1:nbalizas
    idx = find(id == i);
    if isempty(idx)
        theta = NaN;
%         d = NaN;
    else
        theta = ang(idx);
%         d = dist(idx);
    end
    idx = [];
    Z = [Z;
        theta];
    %d];
end
end