clear all
% Fuertemente inspirado de este ejemplo
% openExample('nav/PlanPathBetweenTwoSE2StatesExample')


%% Definicion de variables
resolucion = 0.1; % Metros
inflacion = 0.25; % Lo que crece el mapa para que no toque las paredes el robot
X_limits = [0,10]; %limites del mapa
Y_limits = [0,21];  

% Paredes del mapa
paredes = [ [0 0], [0 19.4];
           [0 19.4], [8.0 19.4];
            [8.0 0],[8.0 19.4];
            [0 0],[8 0];
            [2.0 0],[2.0 3.2];
            [2.0 4.6],[2.0 5.0];
            [2.0 6.4],[2.0 8.6];
            [2.0 4.6],[8.0 4.6];
            [2.0 6.8],[8.0 6.8];
            [1.4 10.0],[8.0 10.0];
            [1.4 14.6],[8.0 14.6]];

%% Creacion mapa

% Para crear un mapa de celdillas de la resolucion deseada dividimos entre
% la resolucion
mapa = zeros((Y_limits(2)-Y_limits(1))/resolucion,(X_limits(2)-X_limits(1))/resolucion);

% Ponemos a 1 todas las paredes del mapa de ocupacion
% Como matlab es especial y todos los indices empiezan en uno he desplazado
% todo 1/resolucion por tanto despues lo que salga del planificador habrá
% que bajarle eso
for i = 1:size(paredes,1)
    % Paredes verticales
    if(paredes(i,1)==paredes(i,3))
        % Calculo indices matriz. El cast es para que matlab no llore
        from_y = cast(paredes(i,2)/resolucion,'uint8')+ 1/resolucion - inflacion/resolucion;
        to_y = cast(paredes(i,4)/resolucion,'uint8')+ 1/resolucion + inflacion/resolucion;
        x = cast(paredes(i,1)/resolucion,'uint8') + 1/resolucion;

        mapa(from_y:to_y,x-inflacion/resolucion:x+inflacion/resolucion) = 1;
    % Paredes horizontales
    elseif (paredes(i,2)==paredes(i,4))
        % Calculo indices matriz. El cast es para que matlab no llore
        y = cast(paredes(i,2)/resolucion,'uint8') + 1/resolucion;
        from_x = cast(paredes(i,1)/resolucion,'uint8') + 1/resolucion - inflacion/resolucion;
        to_x = cast(paredes(i,3)/resolucion,'uint8') + 1/resolucion + inflacion/resolucion;

        mapa(y-inflacion/resolucion:y+inflacion/resolucion,from_x:to_x) = 1;
    end
end

% El mapa se estaba creando de arriba abajo y el occupancy map pone los
% indices al reves, asi que le he hecho un flip.
mapa = flip(mapa,1);

% Creamos mapa de estado y validador
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

% Transformamos mapa hecho a mano a mapa de ocupación que es lo que
% entiende el resto de funciones
map = occupancyMap(mapa);


% Pasamos el mapa al validador 
sv.Map = map;
sv.ValidationDistance = 0.01; % Este parametro aun no se que hace

ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];


% Elegimos el planificador
planner = plannerRRTStar(ss,sv);   % Se puede cambiar a plannerRRT(ss,sv);
planner.MaxConnectionDistance = 1; % Si incrementamos este valor explora más
                                   % si es pequeño a veces no encuentra
                                   % solucion

% Principio y meta. Adecuados a la resolucion.                                   
start = [2/resolucion,2/resolucion,0];
goal =  [8/resolucion,7/resolucion,0];


% Planificador planificando
[pthObj,solnInfo] = planner.plan(start,goal);

% Mostrar el arbol y el recorrido
map.show; hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'b.-');       % arbol
plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2) % path