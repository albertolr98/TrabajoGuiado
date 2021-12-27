function [Xk1, Pk1, debug] = ConLaKalman(Xk, Pk, v, robot, entorno)
% Esta función realiza el filtro de Kalman sobre las medidas
% tomadas a través de los sensores del robot para la corrección de la
% posición del mismo.

% Esto es una solución cutre, habría que hacer algo mejor
Rxini = 0.001;
Ryini = 0.001;
Rthetaini = 0.001;
Rk = [Rxini 0 0; 0 Ryini 0 ; 0 0 Rthetaini];


%% Varianza del ruido del proceso
load('calibracion_odometria.mat', 'Q_pu');
Qk = Matriz_Q(v, Q_pu);

%% Odometría
[Xk_, Pk_] = GetPositionFromOdometry(Xk, Pk, Qk); % X(k+1|k) y P(k+1|k)

%% Medida de los ultrasonidos
[Zk] = apoloGetAllultrasonicSensors('Marvin')';

%% Prediccion de la medida de los ultrasonidos
robot = robot.actualizar_posicion(Xk_);
[Zk_, Hk, ~] = robot.estimar_medidas(entorno);    

%% Comparacion entre predicción y estimación
nu = Zk-Zk_;
debug = Zk_;

% Eliminación de medidas de ultrasonidos fuera de su rango de aplicación
for i = 1:numel(Zk,1)
    if Zk(i) > 2.9 || Zk_(i) > 2.9
        nu(i) = 0;
    end
end

% Matrices Sk y Wk
Sk = Hk*Pk_*((Hk)') + Rk;
Wk = Pk_*((Hk)')*inv(Sk);

%% Corrección del estado X(k+1) y de la matriz P(k+1)
Xk1 = Xk_ + Wk*nu;
Pk1 = (eye(3)-Wk*Hk)*Pk_;

end