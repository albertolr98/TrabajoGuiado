function  MoveRobot(next_position,velocidad, tolerancia)
%MOVEROBOT Esta función toma la información odometrica del robot para
%intentar alcanzar el objetivo.
%     next_position = [x y angle]
%     velocidad = [v_avance v_giro]
%     tolerancia = [tol_distancia tol_giro] 
%                   maximo error admisible para considerar que se ha 
%                   alcanzado el objetivo

%Para lograr este objetivo:
% 1. Calculamos distancia y angulo al objetivo
% 2. Giramos hasta que el robot iguale el ángulo 
% 3. Avanzamos hasta el objetivo
% 4. Giramos para adoptar el ángulo deseado
% 5. Celebramos.

global time_unit % ahora es una variable global; no olvidar ejecutar el script variables globales

    robot = 'Marvin'; 
%     time_unit = 0.01;
    
    %Esto pa testear no vale
    apoloPlaceMRobot(robot,[-4,3,0],pi/2)
    apoloResetOdometry(robot,[-4,3,pi/2])
    
    
% 1. Calculo distancia y angulo al objetivo
    actual_pos = apoloGetOdometry(robot);
    pos_dif = next_position - actual_pos;
    
    
    angle = wrapToPi(atan2(pos_dif(2),pos_dif(1)));
    
    error_angulo = angle - wrapToPi(actual_pos(3));
    error_acum = error_angulo;
    
    disp('angulo:')
    disp(rad2deg(angle))

    disp('error')
    disp(error_angulo)

% 2. Giro en la direccion
    
    %De momento un pid solo proporcional jaja
    %pero hace el apaño
    kp = 100;    
    pid_val = (kp*error_angulo);
    
    
    while abs(error_angulo)>tolerancia(2)
        apoloMoveMRobot(robot, [0 pid_val * velocidad(2)],time_unit);
        apoloUpdate();
        
        actual_pos = apoloGetOdometry(robot);
        angle = wrapToPi(atan2(pos_dif(2),pos_dif(1)));
        error_angulo = angle - wrapToPi(actual_pos(3));
        pid_val = (kp*error_angulo);
        error_acum = [error_acum; error_angulo];
    end
    figure('Name','Error en giro');
    plot(1:size(error_acum),error_acum);
    
    
    
    
% 2. Avance en la direccion

    actual_pos = apoloGetOdometry(robot);
    pos_dif = next_position - actual_pos;
    
    distancia = sqrt(pos_dif(1)^2+pos_dif(2)^2);
    distancia_acumulada = pos_dif(1);
    
    %esta mierda es mas jodida que lo del angulo, con la distancia no es 
    %tan facil hacer control por ser siempre positiva, he pensado en hacer 
    %una distancia negativa teniendo en cuenta el angulo con el que ves la
    %distancia pero aun no lo he logrado
    
   
    
    while (abs(distancia) > tolerancia(1))&& distancia>0
         apoloMoveMRobot(robot,[pid_val*velocidad(1) 0],time_unit);
         apoloUpdate();
        
         
        
       
         
    end
    

end
