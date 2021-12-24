function [X_k1_k, P_k1_k] = KalmanAdaptado(X_k_k, P_k_k, Q, w_r, w_l)

% 1) Con odometría sacas x(k+1|k), a partir de x(k|k)
% 2) Sacas z estimado(k+1) a partir de x(k+1|k) (esto ya lo tengo yo programado en el robot ese que cree)
% 3) Tienes z real (k+1)
% 4) Haces la resta, sacas nu (innovación en la medida)
% 5) Con eso, y según se vio en el EKF, se saca x(k+1|k+1)

% PREDICCIÓN (EFK): Con los valores de la v y la ω se calcula la matriz Q(k) (con Q_pu, que se ha obtenido de la calibración) -> Función Matriz_Q ya programada

% A partir de las medidas de la odometría, que se pueden hacer fuera o dentro de la siguiente función, se calcula la posición nueva del robot X(k+1|k),
% así como la nueva matriz P(k+1|k) -> función GetPositionFromOdometry ya programada

% Calcular la Z estimada. Para ello, se empieza por darle al robot (me refiero al robot de la clase creada de Matlab tan chula) la posición nueva estimada 
% con la función robot = actualizar_posicion(robot, X(k+1|k)). A continuación, se usa la función [z, H] = estimar_medidas(robot, entorno) para obtener la z estimada
% y el jacobiano H. Esto ya está programado, pero hay que meterlo al bucle que WILLY tiene que hacer (filtro de Kalman).

% COMPARACIÓN (EFK): Hacer todas las cosas de Kalman, que son 3 líneas de código, es decir, matriz S (usando la matriz R de la calibración que vamos a tener que inventarnos),
% matriz W y vector de innovación en la medida ν.

    %% Varianza del ruido del proceso
    load('calibracion_odometria.mat', 'Q_pu');
    Qk_1 = Matriz_Q(velocidad, Q_pu);
    
    
    %% Algoritmo     
    Qk_1 = Matriz_Q(velocidad, Q_pu);
    [Zk, Pk] = GetPositionFromOdometry(Xk, Pk, Q);


    % Para acortar el nombre de la variable
    Uk = [trayectoriaDRuido(l); trayectoriaBRuido(l)]; % [delta lineal; delta angular]
    
    %% Nuevo ciclo, k-1 = k.
    Xk_1 = Xk;
    Pk_1 = Pk;
    
    %% Prediccion del estado
    % FALTA ESTO
    X_k = [(Xk_1(1) + Uk(1)*cos(Xk_1(3)+(Uk(2)/2))); % X(k+1|k)
           (Xk_1(2) + Uk(1)*sin(Xk_1(3)+(Uk(2)/2)));
           (Xk_1(3) + Uk(2))];
    Ak = [1 0 (-Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          0 1 (Uk(1)*cos(Xk_1(3)+Uk(2)/2));
          0 0 1                             ];
    Bk = [(cos(Xk_1(3)+Uk(2)/2)) (-0.5*Uk(1)*sin(Xk_1(3)+Uk(2)/2));
          (sin(Xk_1(3)+Uk(2)/2)) (0.5*Uk(1)*cos(Xk_1(3)+Uk(2)/2));
           0                     1                                 ];
    P_k = Ak*Pk_1*((Ak)') + Bk*Qk_1*((Bk)');

    %% Prediccion de la medida
    [Zk_, Hk] = robot.estimar_medidas(obj, X_k);    

    %% Comparacion
    Yk = Zk-Zk_;
    for r=1:3
        if Yk(r)>pi
            Yk(r) = Yk(r) - 2*pi;
        end
        if Yk(r)<(-pi)
            Yk(r) = Yk(r) + 2*pi;
        end
    end
    Sk = Hk*P_k*((Hk)') + Rk;
    Wk = P_k*((Hk)')*inv(Sk);

    % Correccion
    Xk = X_k + Wk*Yk;
    Pk = (eye(3)-Wk*Hk)*P_k;
    
    %Solo para almacenarlo
    Xestimado(:,l) = Xk;
    Pacumulado(1,l) = Pk(1,1);
    Pacumulado(2,l) = Pk(2,2);
    Pacumulado(3,l) = Pk(3,3);    
end