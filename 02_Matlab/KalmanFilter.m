function [X_k1_k1, P_k1_k1,nu] = KalmanFilter(X_k_k, P_k_k, v, robot, entorno)
%[X_k1_k1, P_k1_k1] = KALMANFILTER(X_k_k, P_k_k, v, robot, entorno)
% Esta función realiza el filtro de Kalman sobre las medidas
% tomadas a través de los sensores del robot para la corrección de la
% posición del mismo.

global robot_name laser_name nbalizas %#ok<*GVMIS> 

load('calibracion_odometria.mat', 'Q_pu');
load('calibracion_sensores', 'R');

%% Varianza del ruido del proceso
Qk = Matriz_Q(v, Q_pu);

%% Odometría
[X_k1_k, P_k1_k] = GetPositionFromOdometry(X_k_k, P_k_k, Qk); % X(k+1|k) y P(k+1|k)
robot = robot.actualizar_posicion(X_k1_k);


%% Prediccion de la medida de los ultrasonidos
[Z_estimado, Hk, ~] = robot.estimar_medidas(entorno);    


%% Medida de los ultrasonidos
Z1_k = GetUltrasonicSensorsWithNoise(robot_name);

%% Medida de las balizas (si hay un láser)
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

% Los ángulos se ponen entre -pi y pi
for i = length(Z1_k)+1:length(Z_k)
    nu(i) = wrapToPi(nu(i));
end

% Matrices Sk y Wk
Sk = Hk*P_k1_k*((Hk)') + R;
Wk = P_k1_k*((Hk)')/(Sk);

%% Corrección del estado X(k+1) y de la matriz P(k+1)
X_k1_k1 = X_k1_k + Wk*nu;
P_k1_k1 = (eye(3)-Wk*Hk)*P_k1_k;

%% TRAMPAS para comprobar buen funcionamiento de otras cosas
% aa = apoloGetLocationMRobot(robot_name);
% X_k1_k1 = aa([1 2 4])';
end