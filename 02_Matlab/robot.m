classdef robot
    %ROBOT([x y theta])
    %   Robot móvil que va a ser usado. De momento, va a ser solo un
    %   conjunto de sensores unidos a un cuerpo central :). No sé cómo se
    %   podrá simular el movimiento aquí, de momento.
    
    properties
        sensores
        X % [x; y; theta]
    end
    
    methods
        function obj = robot(X)
            %ROBOT([x y theta])
            %   Constructor del robot, en una posición [x y], girado un
            %   ángulo theta.
            
            if isrow(X)
                X = X';
            end
            
            X(3) = wrapToPi(X(3)); 
            
            obj.sensores = [];
            
            obj.X = X;
        end
        
        function obj = add_us(obj, X_rel)
            %ADD_US(obj, [x_rel y_rel theta_rel])
            %   Añade un sensor ultrasonido con unas posiciones [x_rel y_rel]
            %   respecto al centro del robot, apuntando en un ángulo
            %   theta_rel.
            obj.sensores = [obj.sensores sensor_us(X_rel)];
            obj.sensores(end) = actualizar_posicion(obj.sensores(end), obj.X);
        end
        
        function plot_robot(obj)
        	%PLOT_ROBOT(obj) 
            %   Dibuja el robot y sus sensores. 
            
            x = obj.X(1);
            y = obj.X(2);
            theta = obj.X(3);
            
            x2 = x + 10*cos(theta); % para ver cuál es la parte delantera
            y2 = y + 10*sin(theta);
            
            hold on
            plot(x, y, 'hk', 'MarkerSize', 20); % robot
            plot([x x2], [y y2], '-g'); % dirección de mirada
            
            for i = 1:length(obj.sensores)
                plot_us(obj.sensores(i));
            end
            hold off
        end
        
        function obj = actualizar_posicion(obj, X)
            %obj = ACTUALIZAR_POSICION(obj, X)
            % cambia la posición del robot a X
            
            if isrow(X)
                X = X';
            end
            
            X(3) = wrapToPi(X(3)); 
            
            obj.X = X;
            
            for i = 1:length(obj.sensores)
                obj.sensores(i) = obj.sensores(i).actualizar_posicion(X);
            end
            
        end

        %% Estimación de medidas y Jacobiano
        function [z, H, X_m] = estimar_medidas(obj, entorno)
            %[z, H, X_m] = ESTIMAR_MEDIDAS(obj, entorno)
            %   Devuelve la z estimada de todos los sensores, así como el
            %   jacobiano de estas medidas (H). Devuelve también las 
            %   coordenadas del punto X_m respecto al que están midiendo la 
            %   distancia, para poder ponerlo en una imagen, o donde sea.
            z = zeros(length(obj.sensores), 1);
            H = zeros(length(obj.sensores), 3);
            X_m = zeros(length(obj.sensores), 2);
            
            for i = 1:length(obj.sensores)
                [new_z, new_H, new_X_p] = estimar_medidas(obj.sensores(i), entorno);
                z(i) = new_z;
                H(i,:) = new_H;
                X_m(i,:) = [new_X_p(1) new_X_p(2)];
            end
        end
    end
end

