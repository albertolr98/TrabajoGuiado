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
            
            X(3) = mod(X(3), 2*pi); 
            
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
            
            x2 = x + 100*cos(theta); % para ver cuál es la parte delantera
            y2 = y + 100*sin(theta);
            
            hold on
            plot(x, y, 'hk', 'MarkerSize', 20); % robot
            plot([x x2], [y y2], '-g'); % dirección de mirada
            
            for i = 1:length(obj.sensores)
                plot_us(obj.sensores(i));
            end
            hold off
        end

        %% Estimación de medidas y Jacobiano
        function [z, H] = estimar_medidas(obj, entorno)
            %[z, H] = ESTIMAR_MEDIDAS(obj, entorno)
            %   Devuelve la z estimada de todos los sensores, así como el
            %   jacobiano de estas medidas (H).
            z = zeros(length(obj.sensores), 1);
            H = zeros(length(obj.sensores), 3);
            
            for i = 1:length(obj.sensores)
                [new_z, new_H] = estimar_medidas(obj.sensores(i), entorno, obj);
                z(i) = new_z;
                H(i,:) = new_H;
            end
        end
    end
end

