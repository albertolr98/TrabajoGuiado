function [X_k1, P_k1] = ConLaKalman(Xk, Pk, v, robot, entorno)
% Kalman time
Pxini = 0.001;
Pyini = 0.001;
Pthetaini = 0.001;
Rk = [Pxini 0 0; 0 Pyini 0 ; 0 0 Pthetaini];


%% Varianza del ruido del proceso
load('calibracion_odometria.mat', 'Q_pu');
Qk_1 = Matriz_Q(v, Q_pu);

%% Odometría
[X_k, P_k] = GetPositionFromOdometry(Xk, Pk, Qk_1); % X(k+1|k) y P(k+1|k)

%% Medida de los ultrasonidos
[Zk] = apoloGetAllultrasonicSensors('Marvin')';

%% Prediccion de la medida
[Zk_, Hk, ~] = robot.estimar_medidas(entorno);    

%% Comparacion
nu = Zk-Zk_;
for i = 1:numel(Zk,1)
    if Zk(i) > 2.9 || Zk_(i) > 2.9
        nu(i) = 0;
    end
end

Sk = Hk*P_k*((Hk)') + Rk;
Wk = P_k*((Hk)')*inv(Sk);

% Correccion
X_k1 = X_k + Wk*nu;
P_k1 = (eye(3)-Wk*Hk)*P_k;

% figure("Name", "Posicion 2d");
% plot(pos_robot_array(:,1),pos_robot_array(:,2),'b-');
% title("Trayectoria 2d");
%  
end