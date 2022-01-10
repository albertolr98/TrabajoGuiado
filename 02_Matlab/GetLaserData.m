function Z = GetLaserData(laser_name, nbalizas)
%Z=GETLASERDATA(laser_name, nbalizas)
% Obtiene las medidas de las balizas (apoloGetLaserLandMarks) 

laser = apoloGetLaserLandMarks(laser_name);
id = laser.id;
ang = laser.angle;
% dist = laser.distance;
Z = [];
for i = 1:nbalizas
    idx = find(id == i);
    if isempty(idx)
        phi = NaN;
%         d = NaN;
    else
        phi = ang(idx);
%         d = dist(idx);
    end
    idx = [];
%     Z = [Z;
%         phi; d];
    Z = [Z;
        phi];
end
end