function [X_k1_k1, P_k1_k1] = KalmanFilter(X_k_k, P_k_k, v, robot, entorno)
%[Xk1, Pk1] = KALMANFILTER(Xk, Pk, v, robot, entorno)
% Esta función realiza el filtro de Kalman sobre las medidas
% tomadas a través de los sensores del robot para la corrección de la
% posición del mismo.

global robot_name laser_name nbalizas %#ok<*GVMIS> 

load('calibracion_odometria.mat', 'Q_pu');
load('calibracion_sensores', 'R');
% if isa(robot.sensores(end),'sensor_ls')
%     errores =[ones(1,5)*8.73435e-04,ones(1,20)*0.001];
%     R = diag(errores); % HAY QUE CAMBIAR ESTO
% else
%     R = eye(5)*8.73435e-04;
% end
%% Varianza del ruido del proceso
Qk = Matriz_Q(v, Q_pu);

%% Odometría
[X_k1_K, P_k1_K] = GetPositionFromOdometry(X_k_k, P_k_k, Qk); % X(k+1|k) y P(k+1|k)
robot = robot.actualizar_posicion(X_k1_K);

%% Prediccion de la medida de los ultrasonidos
[Z_estimado, Hk, X_m] = robot.estimar_medidas(entorno);    

%% Medida de los ultrasonidos
Z1_k = GetUltrasonicSensorsWithNoise(robot_name);
% Z1_k = apoloGetAllultrasonicSensors(robot_name)';

%% Medida de las balizas
if isa(robot.sensores(end),'sensor_ls')
    Z2_k = GetLaserData(laser_name, nbalizas);
end

%% Medida de todos los sensores
if isa(robot.sensores(end),'sensor_ls')
    Z_k = [Z1_k; Z2_k];
else
    Z_k = Z1_k;
end

% ver_entorno_y_medidas;

%% Comparacion entre predicción y estimación
nu = Z_k-Z_estimado;

% Eliminación de medidas de ultrasonidos fuera de su rango de aplicación
for i = 1:length(Z1_k) 
    if Z_k(i) > 2.9 || Z_estimado(i) > 2.9
        nu(i) = 0;
    end
end

% Eliminación de medidas de láser cuando no se ven las balizas
for i = length(Z1_k)+1:length(Z_k) 
    if isnan(nu(i))
        nu(i) = 0;
    end
end

% Los ángulos(medidas impares del láser) se ponen entre -pi y pi
for i = length(Z1_k)+1:2:length(Z_k)-1
    if isnan(nu(i))
        nu(i) = wrapToPi(nu(i));
    end
end

% nu

% Matrices Sk y Wk
Sk = Hk*P_k1_K*((Hk)') + R;
Wk = P_k1_K*((Hk)')/(Sk);

%% Corrección del estado X(k+1) y de la matriz P(k+1)
X_k1_k1 = X_k1_K + Wk*nu;
P_k1_k1 = (eye(3)-Wk*Hk)*P_k1_K;

%% TRAMPAS para comprobar buen funcionamiento de otras cosas
% aa = apoloGetLocationMRobot(robot_name);
% X_k1_k1 = aa([1 2 4])';
end