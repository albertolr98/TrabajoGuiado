function ref_pos = Planner(start, goal,resolucion,inflacion, entorno)
%ref_pos = PLANNER(start, goal,resolucion,inflacion, entorno)
% Fuertemente inspirado de este ejemplo
% openExample('nav/PlanPathBetweenTwoSE2StatesExample')
%% Definicion de variables

 % Inflacion: Lo que crece el mapa para que no toque las paredes el robot
X_limits = [0,10]; %limites del mapa
Y_limits = [0,21];  

% Paredes del mapa
p = entorno.paredes;
paredes = zeros(length(p), 4);
for i =1:length(p)
    paredes(i,:) = [p(i).X1' p(i).X2'];
end


angle_start = start(3);
angle_goal = goal(3);

start = (start+1) /resolucion;
goal = (goal+1) / resolucion;

start(3)=angle_start;
goal(3) = angle_goal;

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
planner.MaxConnectionDistance = 3; % Si incrementamos este valor explora más
                                   % si es pequeño a veces no encuentra
                                   % solucion
planner.ContinueAfterGoalReached = true;


% Principio y meta. Adecuados a la resolucion.                                   



% Planificador planificando
[pthObj,solnInfo] = planner.plan(start,goal);

% Mostrar el arbol y el recorrido
figure();
map.show; hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'b.-');       % arbol
plot(pthObj.States(:,1), pthObj.States(:,2),'r-','LineWidth',2) % path

angles_path = pthObj.States(:,3);
ref_pos = ((pthObj.States-1/resolucion)*resolucion);
ref_pos(:,3)= wrapToPi(angles_path);
ref_pos = ref_pos';
end
