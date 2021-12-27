function [X_k1_k1, P_k1_k1] = KalmanFilter(X_k_k, P_k_k, v, robot, entorno)
%[Xk1, Pk1] = KALMANFILTER(Xk, Pk, v, robot, entorno)
% Esta función realiza el filtro de Kalman sobre las medidas
% tomadas a través de los sensores del robot para la corrección de la
% posición del mismo.

global robot_name

load('calibracion_odometria.mat', 'Q_pu');
load('calibracion_sensores', 'R');

%% Varianza del ruido del proceso
Qk = Matriz_Q(v, Q_pu);

%% Odometría
[X_k1_K, P_k1_K] = GetPositionFromOdometry(X_k_k, P_k_k, Qk); % X(k+1|k) y P(k+1|k)
robot = robot.actualizar_posicion(X_k1_K);

%% Prediccion de la medida de los ultrasonidos
[Z_estimado, Hk, ~] = robot.estimar_medidas(entorno);    

%% Medida de los ultrasonidos
Z_k = GetUltrasonicSensorsWithNoise(robot_name);

%% Comparacion entre predicción y estimación
nu = Z_k-Z_estimado;

% Eliminación de medidas de ultrasonidos fuera de su rango de aplicación
for i = 1:length(Z_k) %numel no funciona así
    if Z_k(i) > 2.9 || Z_estimado(i) > 2.9
        nu(i) = 0;
    end
end

% Matrices Sk y Wk
Sk = Hk*P_k1_K*((Hk)') + R;
Wk = P_k1_K*((Hk)')/(Sk);

%% Corrección del estado X(k+1) y de la matriz P(k+1)
X_k1_k1 = X_k1_K + Wk*nu;
P_k1_k1 = (eye(3)-Wk*Hk)*P_k1_K;

end